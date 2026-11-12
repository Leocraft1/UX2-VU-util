/*
 * ux2_vu_daemon.c
 *
 * Legge l'audio in riproduzione (monitor del sink di default PipeWire),
 * calcola il picco per finestre di ~45ms, lo converte in dBFS e lo mappa
 * sui VU meter analogici della Line6 UX2 tramite USB control transfer.
 *
 * Dipendenze:
 *   libpipewire-0.3-dev
 *   libusb-1.0-0-dev
 *
 * Build:
 *   gcc ux2_vu_daemon.c -o ux2_vu_daemon \
 *       $(pkg-config --cflags --libs libpipewire-0.3) -lusb-1.0 -lpthread -lm
 *
 * Uso:
 *   ./ux2_vu_daemon
 *   (Ctrl+C per uscire)
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>

#include <libusb-1.0/libusb.h>
#include <pipewire/pipewire.h>
#include <spa/param/audio/format-utils.h>
#include <spa/utils/result.h>

/* ---------------------------------------------------------------------
 * USB section (Line6 UX2) - same logic as the CLI tool already written
 * --------------------------------------------------------------------- */

#define VID 0x0e41
#define PID 0x4151

#define BM_REQUEST_TYPE 0x40
#define B_REQUEST 0x67

#define VU_WVALUE 0x0601
#define CLIP_WVALUE 0x0201

#define VU_MIN 0
#define VU_MAX 140

#define REF_RATE_MS 50
#define RECONNECT_MS 500

static libusb_context *usb_ctx = NULL;

/* Looks for the UX2 on the bus and opens it. On failure returns NULL and
 * stores the libusb error code in *err (LIBUSB_ERROR_NO_DEVICE if the card
 * is simply not plugged in). */
static libusb_device_handle *open_ux2(int *err){
    libusb_device **list;
    libusb_device_handle *handle = NULL;
    int found = 0;
    ssize_t n, i;

    *err = 0;

    n = libusb_get_device_list(usb_ctx, &list);
    if (n < 0)
    {
        *err = (int)n;
        return NULL;
    }

    for (i = 0; i < n; i++)
    {
        struct libusb_device_descriptor desc;
        int r;

        if (libusb_get_device_descriptor(list[i], &desc) != 0)
            continue;
        if (desc.idVendor != VID || desc.idProduct != PID)
            continue;

        found = 1;
        r = libusb_open(list[i], &handle);
        if (r != 0)
        {
            handle = NULL;
            *err = r;
        }
        break;
    }

    libusb_free_device_list(list, 1);

    if (!found)
        *err = LIBUSB_ERROR_NO_DEVICE;

    return handle;
}

/* Both send functions return 0 on success or a negative libusb error code.
 * They never print: the caller decides what to log, so a missing device
 * does not flood the log with one message per transfer. */
static int send_vu(libusb_device_handle *dev, int left, int right){
    uint16_t wIndex;
    int r;

    if (left < VU_MIN) left = VU_MIN;
    if (left > VU_MAX) left = VU_MAX;
    if (right < VU_MIN) right = VU_MIN;
    if (right > VU_MAX) right = VU_MAX;

    /* wIndex = RRLL */
    wIndex = ((uint16_t)right << 8) | (uint16_t)left;

    r = libusb_control_transfer(
        dev, BM_REQUEST_TYPE, B_REQUEST,
        VU_WVALUE, wIndex, NULL, 0, 1000);

    return r < 0 ? r : 0;
}

//Drives Clip LEDs: 1=none 2=left 3=right 4=both
static int send_clip(libusb_device_handle *dev, int val){
    int r = libusb_control_transfer(
        dev, BM_REQUEST_TYPE, B_REQUEST,
        CLIP_WVALUE, val, NULL, 0, 1000);

    return r < 0 ? r : 0;
}

/* ---------------------------------------------------------------------
 * LEVELS CONVERSION SECTION - Translates values into UX2 VU values
 * FORMULA: value = A * 10^(dB/20) + B*dB + C
 * --------------------------------------------------------------------- */

#define CAL_A 49.28
#define CAL_B 1.252
#define CAL_C 87.59

static int dbfs_to_vu(double dbfs){
    double v;

    if (dbfs < -67.0){
        //Needle is off scale
        return 0;
    }

    v = CAL_A * pow(10.0, dbfs / 20.0) + CAL_B * dbfs + CAL_C;

    if (v < 0) v = 0;
    if (v > VU_MAX) v = VU_MAX;

    return (int)lround(v);
}

static double peak_to_dbfs(float peak){
    if (peak <= 0.0f)
    {
        return -120.0; /* absolute silence */
    }
    return 20.0 * log10((double)peak);
}

/* ---------------------------------------------------------------------
 * AUDIO SECTION (PipeWire) - Captures from deafult sink monitor
 * --------------------------------------------------------------------- */

static _Atomic float g_peak_left = 0.0f;
static _Atomic float g_peak_right = 0.0f;
static _Atomic int g_running = 1;

//Update peak only if higher than previous 
static void atomic_update_peak(_Atomic float *slot, float value){
    float cur = atomic_load_explicit(slot, memory_order_relaxed);
    while (value > cur && !atomic_compare_exchange_weak_explicit(slot, &cur, value,memory_order_relaxed, memory_order_relaxed)){/* cur is updated by compare_exchange, retry */}
}

struct pw_data{
    struct pw_main_loop *loop;
    struct pw_stream *stream;
};

static uint64_t g_process_calls = 0;

static void on_process(void *userdata){
    struct pw_data *data = userdata;
    struct pw_buffer *b;
    struct spa_buffer *buf;
    float *samples;
    uint32_t n_samples;
    uint32_t n_channels;
    uint32_t i;

    if ((b = pw_stream_dequeue_buffer(data->stream)) == NULL)
    {
        return;
    }

    buf = b->buffer;
    if (buf->datas[0].data == NULL)
    {
        pw_stream_queue_buffer(data->stream, b);
        return;
    }

    samples = (float *)buf->datas[0].data;
    /* assuming interleaved float32 layout, 2 channels (set in the format) */
    n_channels = 2;
    n_samples = buf->datas[0].chunk->size / sizeof(float) / n_channels;

    float peak_l = 0.0f, peak_r = 0.0f;

    for (i = 0; i < n_samples; i++)
    {
        float l = fabsf(samples[i * n_channels + 0]);
        float r = fabsf(samples[i * n_channels + 1]);
        if (l > peak_l)
            peak_l = l;
        if (r > peak_r)
            peak_r = r;
    }

    atomic_update_peak(&g_peak_left, peak_l);
    atomic_update_peak(&g_peak_right, peak_r);

    /* DEBUG: stampa ogni ~50 chiamate per non intasare il terminale */
    g_process_calls++;
    if (g_process_calls % 50 == 1)
    {
        /*
        fprintf(stderr,
                "[audio] callback #%llu  n_samples=%u  peak_buffer L=%.4f R=%.4f\n",
                (unsigned long long)g_process_calls, n_samples, peak_l, peak_r);*/
    }

    pw_stream_queue_buffer(data->stream, b);
}

static void on_state_changed(void *userdata, enum pw_stream_state old, enum pw_stream_state state, const char *error){
    (void)userdata;
    fprintf(stderr, "[pipewire] stream state: %s -> %s%s%s\n",
        pw_stream_state_as_string(old),
        pw_stream_state_as_string(state),
        error ? "  ERROR: " : "",
        error ? error : "");
}

static const struct pw_stream_events stream_events = {
    PW_VERSION_STREAM_EVENTS,
    .process = on_process,
    .state_changed = on_state_changed,
};

/* ---------------------------------------------------------------------
 * USB COMMUNICATION SECTION - Sends values to UX2
 *
 * The USB thread owns the device handle. It is independent from the
 * PipeWire stream: it waits for the UX2 to show up, plays the startup
 * animation, sends levels, and goes back to waiting if the card is
 * unplugged. Messages are logged only when the state changes.
 * --------------------------------------------------------------------- */

static void log_open_error(int err){
    if (err == LIBUSB_ERROR_NO_DEVICE)
        fprintf(stderr, "[usb] UX2 not found, waiting for it to be connected\n");
    else if (err == LIBUSB_ERROR_ACCESS)
        fprintf(stderr, "[usb] permission denied opening UX2 (check the udev rule), retrying\n");
    else
        fprintf(stderr, "[usb] cannot open UX2: %s, retrying\n", libusb_error_name(err));
}

// Startup animation: every clip LED on and both meters at full scale
static void vu_anim(libusb_device_handle *dev){
    send_clip(dev, 4);
    send_vu(dev, VU_MAX, VU_MAX);

    usleep(800 * 1000);

    send_clip(dev, 1);
    send_vu(dev, 0, 0);
}

/* Shutdown task: sets meters to zero and turns off the clip indicator */
static void vu_zero(libusb_device_handle *dev){
    send_clip(dev, 1);
    send_vu(dev, 0, 0);
}

static void *usb_thread(void *arg){
    libusb_device_handle *dev = NULL;
    int last_err = 0; //last open error already logged
    (void)arg;

    while (atomic_load_explicit(&g_running, memory_order_relaxed)){
        if (!dev){
            int err;

            dev = open_ux2(&err);
            if (!dev){
                if (err != last_err){
                    log_open_error(err);
                    last_err = err;
                }
                usleep(RECONNECT_MS * 1000);
                continue;
            }

            last_err = 0;
            fprintf(stderr, "[usb] UX2 connected (VID=%04x PID=%04x)\n", VID, PID);
            vu_anim(dev);
        }

        //Calculates peak level
        float peak_l = atomic_exchange_explicit(&g_peak_left, 0.0f, memory_order_relaxed);
        float peak_r = atomic_exchange_explicit(&g_peak_right, 0.0f, memory_order_relaxed);

        //Converts to dBs
        double dbfs_l = peak_to_dbfs(peak_l);
        double dbfs_r = peak_to_dbfs(peak_r);

        //Converts to UX2 VU values
        int vu_l = dbfs_to_vu(dbfs_l);
        int vu_r = dbfs_to_vu(dbfs_r);

        //Maps L/R clip state to the 4 states: 1=none 2=left 3=right 4=both
        int clip_l = (peak_l >= 1.0f);
        int clip_r = (peak_r >= 1.0f);
        int clip_stat;
        if (clip_l && clip_r)      clip_stat = 4;
        else if (clip_r)           clip_stat = 3;
        else if (clip_l)           clip_stat = 2;
        else                        clip_stat = 1;

        int r1 = send_clip(dev, clip_stat);
        int r2 = send_vu(dev, vu_l, vu_r);

        if (r1 < 0 || r2 < 0){
            fprintf(stderr, "[usb] UX2 lost (%s), waiting for it to come back\n",
                    libusb_error_name(r1 < 0 ? r1 : r2));
            libusb_close(dev);
            dev = NULL;
            last_err = LIBUSB_ERROR_NO_DEVICE; //already reported, do not repeat
            continue;
        }

        usleep(REF_RATE_MS * 1000);
    }

    if (dev){
        vu_zero(dev);
        libusb_close(dev);
    }

    return NULL;
}

static struct pw_main_loop *g_pw_loop = NULL;

/* ---------------------------------------------------------------------
 * SIGNAL HANDLING - Stores int signal values into atomic
 * --------------------------------------------------------------------- */

static void handle_sigint(int sig){
    (void)sig;
    atomic_store_explicit(&g_running, 0, memory_order_relaxed);
    if (g_pw_loop){
        pw_main_loop_quit(g_pw_loop);
    }
}

int main(int argc, char *argv[]){
    struct pw_data data = {0};
    const struct spa_pod *params[1];
    uint8_t buffer[1024];
    struct spa_pod_builder b = SPA_POD_BUILDER_INIT(buffer, sizeof(buffer));
    pthread_t usb_tid;
    int r;

    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    r = libusb_init(&usb_ctx);
    if (r != 0){
        fprintf(stderr, "libusb_init failed: %s\n", libusb_error_name(r));
        return 1;
    }

    //init PipeWire
    pw_init(&argc, &argv);

    data.loop = pw_main_loop_new(NULL);
    g_pw_loop = data.loop;

    data.stream = pw_stream_new_simple(
        pw_main_loop_get_loop(data.loop),
        "ux2-vu-meter",
        pw_properties_new(
            PW_KEY_MEDIA_TYPE, "Audio",
            PW_KEY_MEDIA_CATEGORY, "Capture",
            PW_KEY_MEDIA_ROLE, "Music",
            /* Automatically captures the default sink's monitor
             * (EasyEffects or UX2, whichever is currently active) */
            PW_KEY_STREAM_CAPTURE_SINK, "true",
            NULL),
        &stream_events,
        &data);

    params[0] = spa_format_audio_raw_build(
        &b, SPA_PARAM_EnumFormat,
        &SPA_AUDIO_INFO_RAW_INIT(
                .format = SPA_AUDIO_FORMAT_F32,
                .channels = 2,
                .rate = 48000));

    r = pw_stream_connect(
        data.stream,
        PW_DIRECTION_INPUT,
        PW_ID_ANY,
        PW_STREAM_FLAG_AUTOCONNECT |
            PW_STREAM_FLAG_MAP_BUFFERS |
            PW_STREAM_FLAG_RT_PROCESS,
        params, 1);

    if (r < 0){
        fprintf(stderr, "[pipewire] pw_stream_connect failed: %s (%d)\n",
                spa_strerror(r), r);
        return 1;
    }

    if (pthread_create(&usb_tid, NULL, usb_thread, NULL) != 0){
        fprintf(stderr, "Unable to create USB thread\n");
        return 1;
    }

    printf("Listening to default sink monitor. Press Ctrl+C to exit.\n");

    pw_main_loop_run(data.loop);

    /* --- shutdown --- */
    atomic_store_explicit(&g_running, 0, memory_order_relaxed);
    pthread_join(usb_tid, NULL); //zeroes the meters and closes the UX2

    pw_stream_destroy(data.stream);
    pw_main_loop_destroy(data.loop);
    pw_deinit();

    libusb_exit(usb_ctx);

    return 0;
}
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
 * Parte USB (Line6 UX2) - stessa logica del programma CLI gia' scritto
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

static libusb_context *usb_ctx = NULL;
static libusb_device_handle *usb_dev = NULL;

static libusb_device_handle *open_ux2(libusb_context *ctx){
    libusb_device_handle *dev = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!dev)
    {
        fprintf(stderr, "Unable to open UX2 device (%04x:%04x)\n", VID, PID);
        return NULL;
    }
    return dev;
}

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

    if (r < 0)
    {
        fprintf(stderr, "VU: control transfer fallito: %s (%d)\n",
                libusb_error_name(r), r);
        return -1;
    }
    return 0;
}

//Drives Clip LEDs
static int send_peak(libusb_device_handle *dev, int val){
    int r = libusb_control_transfer(
        dev, BM_REQUEST_TYPE, B_REQUEST,
        CLIP_WVALUE, val, NULL, 0, 1000);
    if (r < 0)
    {
        fprintf(stderr, "Clip LEDs: control transfer failed: %s (%d)\n",
                libusb_error_name(r), r);
        return -1;
    }
    return 0;
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
        return -120.0; /* silenzio assoluto */
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
    while (value > cur && !atomic_compare_exchange_weak_explicit(slot, &cur, value,memory_order_relaxed, memory_order_relaxed)){/* cur viene aggiornato da compare_exchange, riprova */}
}

struct pw_data{
    struct pw_main_loop *loop;
    struct pw_stream *stream;
    pthread_t sender_tid;
};

static _Atomic int g_sender_started = 0;

static void *vu_sender_thread(void *arg);

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
    /* assumiamo layout interleaved float32, 2 canali (impostato nel format) */
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
    struct pw_data *data = userdata;
    (void)userdata;
    fprintf(stderr, "[pipewire] stream state: %s -> %s%s%s\n",
        pw_stream_state_as_string(old),
        pw_stream_state_as_string(state),
        error ? "  ERRORE: " : "",
        error ? error : "");

    if (state == PW_STREAM_STATE_STREAMING && !atomic_load_explicit(&g_sender_started, memory_order_relaxed)){
        int ur;

        ur = libusb_init(&usb_ctx);
        if (ur != 0){
            fprintf(stderr, "libusb_init failed: %s\n", libusb_error_name(ur));
            if (data->loop)
                pw_main_loop_quit(data->loop);
            return;
        }

        usb_dev = open_ux2(usb_ctx);
        if (!usb_dev)
        {
            fprintf(stderr, "[usb] cannot open UX2 device\n");
            if (data->loop)
                pw_main_loop_quit(data->loop);
            return;
        }
        fprintf(stderr, "[usb] opened UX2 (VID=%04x PID=%04x)\n", VID, PID);

        atomic_store_explicit(&g_sender_started, 1, memory_order_relaxed);
        if (pthread_create(&data->sender_tid, NULL, vu_sender_thread, NULL) != 0)
        {
            fprintf(stderr, "Unable to create USB send thread\n");
        }
        else
        {
            fprintf(stderr, "[usb] audio stream is active\n");
        }
    }
}

static const struct pw_stream_events stream_events = {
    PW_VERSION_STREAM_EVENTS,
    .process = on_process,
    .state_changed = on_state_changed,
};

/* ---------------------------------------------------------------------
 * USB COMMUNICATION SECTION - Sends values to UX2
 * --------------------------------------------------------------------- */

static void *vu_sender_thread(void *arg){
    (void)arg;

    while (atomic_load_explicit(&g_running, memory_order_relaxed)){
        //Calculates peak level
        float peak_l = atomic_exchange_explicit(&g_peak_left, 0.0f, memory_order_relaxed);
        float peak_r = atomic_exchange_explicit(&g_peak_right, 0.0f, memory_order_relaxed);

        //Converts to dBs
        double dbfs_l = peak_to_dbfs(peak_l);
        double dbfs_r = peak_to_dbfs(peak_r);

        //Converts to UX2 VU values
        int vu_l = dbfs_to_vu(dbfs_l);
        int vu_r = dbfs_to_vu(dbfs_r);

        int r1 = send_peak(usb_dev, 2);
        int r2 = send_vu(usb_dev, vu_l, vu_r);

        /* DEBUG */
        /*
        fprintf(stderr,
                "[usb] peak L=%.4f R=%.4f  dBFS L=%.1f R=%.1f  VU L=%d R=%d  "
                "keepalive=%s vu_send=%s\n",
                peak_l, peak_r, dbfs_l, dbfs_r, vu_l, vu_r,
                r1 == 0 ? "ok" : "FALLITA",
                r2 == 0 ? "ok" : "FALLITA");*/

        usleep(REF_RATE_MS * 1000);
    }

    return NULL;
}

//Shutdown task, sets meters to zero and removes clip indicator
static void *vu_anim(){
    int vu_l = 140;
    int vu_r = 140;

    int r1 = send_peak(usb_dev, 2);
    usleep(1000 * 1000);
    int r2 = send_vu(usb_dev, vu_l, vu_r);
    usleep(1000);
    int r3 = send_peak(usb_dev, 1);

    fprintf(stderr,
        "[usb] Started meters\n",
        r1 == 0 ? "ok" : "FALLITA",
        r2 == 0 ? "ok" : "FALLITA");
}

//Shutdown task, sets meters to zero and removes clip indicator
static void *vu_zero(){
    int vu_l = 0;
    int vu_r = 0;

    int r1 = send_peak(usb_dev, 1);
    int r2 = send_vu(usb_dev, vu_l, vu_r);

    fprintf(stderr,
        "[usb] Meters set to zero\n",
        r1 == 0 ? "ok" : "FALLITA",
        r2 == 0 ? "ok" : "FALLITA");
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
    int r;

    signal(SIGINT, handle_sigint);

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
            /* Cattura automaticamente il monitor del sink predefinito
             * (EasyEffects o UX2 in base a cosa e' attivo nel sistema) */
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
        fprintf(stderr, "[pipewire] pw_stream_connect fallito: %s (%d)\n",
                spa_strerror(r), r);
        return 1;
    }
    fprintf(stderr, "[pipewire] pw_stream_connect chiamata con successo, "
                    "in attesa che lo stream si colleghi...\n");

    /* il thread di invio USB parte da solo quando lo stream PipeWire
     * raggiunge lo stato STREAMING (vedi on_state_changed) */
    printf("In ascolto del monitor del sink di default. Ctrl+C per uscire.\n");

    pw_main_loop_run(data.loop);
    //vu_anim();

    /* --- shutdown --- */
    atomic_store_explicit(&g_running, 0, memory_order_relaxed);
    if (atomic_load_explicit(&g_sender_started, memory_order_relaxed)){
        pthread_join(data.sender_tid, NULL);
    }

    pw_stream_destroy(data.stream);
    pw_main_loop_destroy(data.loop);
    pw_deinit();

    vu_zero();

    if (usb_dev){
        libusb_close(usb_dev);
    }
    libusb_exit(usb_ctx);

    return 0;
}

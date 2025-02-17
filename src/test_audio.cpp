#include <iostream>
#include <thread>
#include <queue>
#include <atomic>
#include <condition_variable>
#include <sndfile.h>
#include <chrono>
#include "hl2ss_mt.h"
#include "hl2ss_lnm.h"

// Settings --------------------------------------------------------------------

const char* host = "192.168.50.33";
const uint16_t port = hl2ss::stream_port::MICROPHONE;
const uint8_t profile = hl2ss::audio_profile::AAC_24000;

// Audio format
const int audio_format = (profile == hl2ss::audio_profile::RAW) ? SF_FORMAT_PCM_16 : SF_FORMAT_FLOAT;
const int channels = 2; //hl2ss::parameters_microphone::CHANNELS;
const int sample_rate = 48000; //hl2ss::parameters_microphone::SAMPLE_RATE;
const uint16_t group_size = 4096; //hl2ss::parameters_microphone::GROUP_SIZE_AAC;


// Global control flag
std::atomic<bool> enable(true);

// Audio queue
std::queue<std::vector<float>> pcmqueue;
std::mutex queue_mutex;
std::condition_variable queue_cv;

// Function to handle audio playback
void pcmworker() {
    SF_INFO sfinfo;
    sfinfo.channels = channels;
    sfinfo.samplerate = sample_rate;
    sfinfo.format = SF_FORMAT_WAV | audio_format;
    std::cout << "audio_format: " << audio_format << std::endl;
    std::cout << "channels: " << channels << std::endl;
    std::cout << "sample_rate: " << sample_rate << std::endl;

    SNDFILE* audio_file = sf_open("/project/ws_dev/src/hri_cacti_xr/image_writer/output/test_audio.wav", SFM_WRITE, &sfinfo);
    if (!audio_file) {
        std::cerr << "Failed to open audio file for writing." << std::endl;
        return;
    }

    while (enable) {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [] { return !pcmqueue.empty() || !enable; });

        if (!enable && pcmqueue.empty()) break;

        auto audio_data = pcmqueue.front();
        pcmqueue.pop();
        lock.unlock();

        sf_writef_float(audio_file, audio_data.data(), audio_data.size());
    }

    sf_close(audio_file);
}

int main() {
    std::thread audio_thread(pcmworker);

    std::unique_ptr<hl2ss::rx_microphone> client = hl2ss::lnm::rx_microphone(host, port, profile);
    client->open();

    auto start_time = std::chrono::steady_clock::now();
    auto end_time = start_time + std::chrono::seconds(5);

    while (enable && std::chrono::steady_clock::now() < end_time) {
        auto data = client->get_next_packet();
        // Process each chunk separately
        std::vector<float> audio_data(data->sz_payload / sizeof(float)); // using group_size as the number of samples to read for one channel
        std::cout << "audio_data size: " << audio_data.size() << std::endl;
        hl2ss::map_microphone_aac region = hl2ss::unpack_microphone_aac(data->payload.get());
        std::cout << "region.samples size: " << data->sz_payload / sizeof(float) << std::endl;
        memcpy(audio_data.data(), region.samples, data->sz_payload);
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            pcmqueue.push(audio_data);
        }
        queue_cv.notify_one();

        std::cout << "pushed to queue" << std::endl;
    }

    client->close();

    enable = false;
    queue_cv.notify_all();
    audio_thread.join();

    return 0;
}
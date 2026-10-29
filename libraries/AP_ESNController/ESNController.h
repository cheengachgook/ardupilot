#pragma once
#include "AP_ESNController_config.h"

#include <AP_Param/AP_Param.h>
#include <cmath>
#include <cstdint>
#include <cstring>

// Параметры по умолчанию
#define ESN_DEFAULT_RESERVOIR_SIZE  100
#define ESN_DEFAULT_SPECTRAL_RADIUS 0.9f
#define ESN_DEFAULT_LEAK_RATE       0.05f
#define ESN_DEFAULT_NOISE_LEVEL     0.01f
#define ESN_DEFAULT_REGULARIZATION  1e-6f
#define ESN_DEFAULT_LAMBDA          0.999f

class ESNController {
public:
    ESNController() :
        n_inputs(0),
        n_reservoir(0),
        spectral_radius(0),
        leak_rate(0),
        noise_level(0),
        regularization(0),
        lambda(0),
        W_in(nullptr),
        W_res(nullptr),
        W_out(nullptr),
        state(nullptr),
        extended_state(nullptr),
        P(nullptr),
        input_mean(nullptr),
        input_std(nullptr),
        _u_norm(nullptr),
        _reservoir_scratch(nullptr),
        _noisy_state(nullptr),
        _P_s(nullptr),
        _g(nullptr),
        _sP(nullptr),
        _new_P(nullptr),
        _eig_vec(nullptr),
        _eig_new_vec(nullptr)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    // Метод реальной инициализации
    void init(uint8_t _n_inputs,
              uint8_t _n_reservoir = ESN_DEFAULT_RESERVOIR_SIZE,
              float _spectral_radius = ESN_DEFAULT_SPECTRAL_RADIUS,
              float _leak_rate = ESN_DEFAULT_LEAK_RATE,
              float _noise_level = ESN_DEFAULT_NOISE_LEVEL,
              float _regularization = ESN_DEFAULT_REGULARIZATION,
              float _lambda = ESN_DEFAULT_LAMBDA);

    ~ESNController();

    float update(const float* u);
    void train(float target);
    void reset();
    void pretrain(const float* data, uint16_t num_samples, 
                  float eta = 0.001f, float mu = 0.0f, float sigma = 1.0f, 
                  uint8_t epochs = 1);
    uint16_t load_pretrain_data(const char* filename, float** out_data);

    // ─── AP_Param интерфейс ───
    static const struct AP_Param::GroupInfo var_info[];

    // Применить новые значения параметров без полной переинициализации.
    // Возвращает true, если изменился спектральный радиус (нужен re-init W_res)
    // или размер резервуара (нужен полный re-init).
    bool reload_params();

private:
    uint8_t n_inputs;
    uint8_t n_reservoir;
    float spectral_radius;
    float leak_rate;
    float noise_level;
    float regularization;
    float lambda;
    float* gain;
    float* bias;

    // ─── Параметры, доступные из GCS ───
    AP_Int16  cfg_reservoir_size;
    AP_Float  cfg_spectral_radius;
    AP_Float  cfg_leak_rate;
    AP_Float  cfg_noise_level;
    AP_Float  cfg_regularization;
    AP_Float  cfg_lambda;

    // ─── Постоянные массивы сети ───
    float* W_in;
    float* W_res;
    float* W_out;
    float* state;
    float* extended_state;
    float* P;
    float* input_mean;
    float* input_std;

    // ─── Предварительно выделенные рабочие буферы ───
    // Все аллокации выполняются один раз в init(); в hot path (update/train)
    // динамической памяти не запрашивается.
    float* _u_norm;             // размер n_inputs
    float* _reservoir_scratch;  // размер n_reservoir — рекуррентная часть W_res·state
    float* _noisy_state;        // размер n_reservoir + n_inputs
    float* _P_s;                // размер n_reservoir + n_inputs — P · s
    float* _g;                  // размер n_reservoir + n_inputs — вектор усиления
    float* _sP;                 // размер n_reservoir + n_inputs — s^T · P
    float* _new_P;              // размер (n_reservoir + n_inputs)^2 — новая матрица P
    float* _eig_vec;            // размер n_reservoir — для power method в init_weights
    float* _eig_new_vec;        // размер n_reservoir — для power method в init_weights

    void init_weights();
    void init_rls();
    void normalize(const float* u, float* u_norm);
    void compute_new_state(const float* u_norm);
    void form_extended_state(const float* u_norm);
    float predict() const;

    // ─── Управление памятью ───
    void free_buffers();
    void alloc_buffers();
};
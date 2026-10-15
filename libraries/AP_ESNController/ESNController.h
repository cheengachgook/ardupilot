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
        input_std(nullptr)
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

    // ─── Параметры, доступные из GCS ───
    AP_Int16  cfg_reservoir_size;
    AP_Float  cfg_spectral_radius;
    AP_Float  cfg_leak_rate;
    AP_Float  cfg_noise_level;
    AP_Float  cfg_regularization;
    AP_Float  cfg_lambda;

    // Внутреннее состояние (без изменений)
    float* W_in;
    float* W_res;
    float* W_out;
    float* state;
    float* extended_state;
    float* P;
    float* input_mean;
    float* input_std;

    void init_weights();
    void init_rls();
    void normalize(const float* u, float* u_norm);
    void compute_new_state(const float* u_norm);
    void form_extended_state(const float* u_norm);
    float predict() const;
};
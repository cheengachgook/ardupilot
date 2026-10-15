#include "ESNController.h"
#include <cstdlib>
#include <ctime>

// Префикс ESN_ будет автоматически добавлен родительским классом
const AP_Param::GroupInfo ESNController::var_info[] = {
    AP_GROUPINFO("NRES",   0, ESNController, cfg_reservoir_size, ESN_DEFAULT_RESERVOIR_SIZE),
    AP_GROUPINFO("SR",     1, ESNController, cfg_spectral_radius, ESN_DEFAULT_SPECTRAL_RADIUS),
    AP_GROUPINFO("LR",     2, ESNController, cfg_leak_rate, ESN_DEFAULT_LEAK_RATE),
    AP_GROUPINFO("NOISE",  3, ESNController, cfg_noise_level, ESN_DEFAULT_NOISE_LEVEL),
    AP_GROUPINFO("REG",    4, ESNController, cfg_regularization, ESN_DEFAULT_REGULARIZATION),
    AP_GROUPINFO("LAMBDA", 5, ESNController, cfg_lambda, ESN_DEFAULT_LAMBDA),
    AP_GROUPEND
};

// Равномерное случайное число в диапазоне [-scale, scale]
static float rand_uniform(float scale) {
    return (static_cast<float>(rand()) / RAND_MAX * 2.0f - 1.0f) * scale;
}

// Гауссово случайное число (метод Бокса-Мюллера)
static float rand_gaussian() {
    float u1 = static_cast<float>(rand()) / RAND_MAX;
    float u2 = static_cast<float>(rand()) / RAND_MAX;
    return sqrtf(-2.0f * logf(u1 + 1e-12f)) * cosf(2.0f * M_PI * u2);
}

void ESNController::init(uint8_t _n_inputs, uint8_t _n_reservoir,
                         float _sr, float _lr, float _noise,
                         float _reg, float _lam)
{
    // Освобождаем память, если уже инициализировано
    if (W_in) delete[] W_in;
    if (W_res) delete[] W_res;
    if (W_out) delete[] W_out;
    if (state) delete[] state;
    if (extended_state) delete[] extended_state;
    if (P) delete[] P;
    if (input_mean) delete[] input_mean;
    if (input_std) delete[] input_std;

    // Читаем из AP_Param, если значение задано; иначе используем переданное
    n_inputs = _n_inputs;
    n_reservoir = (cfg_reservoir_size.get() > 0) ? cfg_reservoir_size.get() : _n_reservoir;
    spectral_radius = (cfg_spectral_radius.get() > 0.0f) ? cfg_spectral_radius.get() : _sr;
    leak_rate       = (cfg_leak_rate.get() > 0.0f)      ? cfg_leak_rate.get()       : _lr;
    noise_level     = (cfg_noise_level.get() >= 0.0f)   ? cfg_noise_level.get()     : _noise;
    regularization  = (cfg_regularization.get() > 0.0f) ? cfg_regularization.get()  : _reg;
    lambda          = (cfg_lambda.get() > 0.0f)         ? cfg_lambda.get()          : _lam;

    W_in = new float[n_reservoir * n_inputs];
    W_res = new float[n_reservoir * n_reservoir];
    W_out = new float[n_reservoir + n_inputs];

    state = new float[n_reservoir];
    extended_state = new float[n_reservoir + n_inputs];

    P = new float[(n_reservoir + n_inputs) * (n_reservoir + n_inputs)];

    input_mean = new float[n_inputs];
    input_std = new float[n_inputs];

    srand(time(nullptr));
    init_weights();
    init_rls();

    memset(state, 0, n_reservoir * sizeof(float));
    memset(extended_state, 0, (n_reservoir + n_inputs) * sizeof(float));
    memset(W_out, 0, (n_reservoir + n_inputs) * sizeof(float));

    for (uint8_t i = 0; i < n_inputs; i++) {
        input_mean[i] = 0.0f;
        input_std[i] = 1.0f;
    }
}

ESNController::~ESNController() {
    delete[] W_in;
    delete[] W_res;
    delete[] W_out;
    delete[] state;
    delete[] extended_state;
    delete[] P;
    delete[] input_mean;
    delete[] input_std;
}

// ---------------------------------------------------------------------------
// Инициализация входных и рекуррентных весов
// ---------------------------------------------------------------------------
void ESNController::init_weights() {
    // Входные веса: равномерный шум с масштабом sqrt(1/n_inputs)
    float scale_in = sqrtf(1.0f / n_inputs);
    for (uint16_t i = 0; i < n_reservoir * n_inputs; i++) {
        W_in[i] = rand_uniform(scale_in);
    }

    // Рекуррентные веса: разреженная матрица с нормальным распределением
    float sparsity = 0.1f + 0.1f * expf(-n_reservoir / 100.0f);
    for (uint16_t i = 0; i < n_reservoir * n_reservoir; i++) {
        if (static_cast<float>(rand()) / RAND_MAX < sparsity) {
            W_res[i] = rand_gaussian();
        } else {
            W_res[i] = 0.0f;
        }
    }

    // Масштабирование до заданного спектрального радиуса (power method)
    float* vec = new float[n_reservoir];
    for (uint8_t i = 0; i < n_reservoir; i++) vec[i] = rand_gaussian();
    float prev_norm = 0;
    float max_eig = 1.0f;
    for (int iter = 0; iter < 50; iter++) {
        float* new_vec = new float[n_reservoir];
        for (uint8_t i = 0; i < n_reservoir; i++) {
            float sum = 0;
            for (uint8_t j = 0; j < n_reservoir; j++) {
                sum += W_res[i * n_reservoir + j] * vec[j];
            }
            new_vec[i] = sum;
        }
        float norm = 0;
        for (uint8_t i = 0; i < n_reservoir; i++) norm += new_vec[i] * new_vec[i];
        norm = sqrtf(norm);
        if (iter > 10 && prev_norm > 0) {
            max_eig = norm / prev_norm;
            if (max_eig < 0) max_eig = -max_eig;
            if (max_eig > 1e-4) break;
        }
        prev_norm = norm;
        for (uint8_t i = 0; i < n_reservoir; i++) vec[i] = new_vec[i];
        delete[] new_vec;
    }
    delete[] vec;

    if (max_eig < 1e-6) max_eig = 1.0f;
    float scale = spectral_radius / max_eig;
    for (uint16_t i = 0; i < n_reservoir * n_reservoir; i++) {
        W_res[i] *= scale;
    }
}

// ---------------------------------------------------------------------------
// Инициализация RLS: P = 100 * I
// ---------------------------------------------------------------------------
void ESNController::init_rls() {
    uint16_t ext_size = n_reservoir + n_inputs;
    for (uint16_t i = 0; i < ext_size * ext_size; i++) P[i] = 0;
    for (uint16_t i = 0; i < ext_size; i++) {
        P[i * ext_size + i] = 100.0f;
    }
}

// ---------------------------------------------------------------------------
// Адаптивная нормализация: экспоненциальное скользящее среднее и MAD
// ---------------------------------------------------------------------------
void ESNController::normalize(const float* u, float* u_norm) {
    for (uint8_t i = 0; i < n_inputs; i++) {
        input_mean[i] = 0.99f * input_mean[i] + 0.01f * u[i];
        input_std[i] = 0.99f * input_std[i] + 0.01f * fabsf(u[i] - input_mean[i]) + 1e-6f;
        u_norm[i] = (u[i] - input_mean[i]) / input_std[i];
    }
}

// ---------------------------------------------------------------------------
// Обновление состояния резервуара (leaky integrator + tanh)
// ---------------------------------------------------------------------------
void ESNController::compute_new_state(const float* u_norm) {
    float* input_part = new float[n_reservoir];
    for (uint8_t i = 0; i < n_reservoir; i++) {
        float sum = 0;
        for (uint8_t j = 0; j < n_inputs; j++) {
            sum += W_in[i * n_inputs + j] * u_norm[j];
        }
        input_part[i] = sum;
    }

    float* reservoir_part = new float[n_reservoir];
    for (uint8_t i = 0; i < n_reservoir; i++) {
        float sum = 0;
        for (uint8_t j = 0; j < n_reservoir; j++) {
            sum += W_res[i * n_reservoir + j] * state[j];
        }
        reservoir_part[i] = sum;
    }

    for (uint8_t i = 0; i < n_reservoir; i++) {
        float activation = tanhf(input_part[i] + reservoir_part[i]);
        state[i] = (1 - leak_rate) * state[i] + leak_rate * activation;
    }

    delete[] input_part;
    delete[] reservoir_part;
}

// ---------------------------------------------------------------------------
// Формирование расширенного состояния [state; u_norm]
// ---------------------------------------------------------------------------
void ESNController::form_extended_state(const float* u_norm) {
    memcpy(extended_state, state, n_reservoir * sizeof(float));
    memcpy(extended_state + n_reservoir, u_norm, n_inputs * sizeof(float));
}

// ---------------------------------------------------------------------------
// Предсказание: W_out * extended_state
// ---------------------------------------------------------------------------
float ESNController::predict() const {
    float out = 0;
    uint16_t ext_size = n_reservoir + n_inputs;
    for (uint16_t i = 0; i < ext_size; i++) {
        out += W_out[i] * extended_state[i];
    }
    return out;
}

// ---------------------------------------------------------------------------
// Основной метод обновления: нормализация, новое состояние, предсказание
// ---------------------------------------------------------------------------
float ESNController::update(const float* u) {
    float* u_norm = new float[n_inputs];
    normalize(u, u_norm);

    compute_new_state(u_norm);
    form_extended_state(u_norm);

    delete[] u_norm;

    return predict();
}

// ---------------------------------------------------------------------------
// RLS-обучение: корректирует W_out по ошибке target - prediction
// ---------------------------------------------------------------------------
void ESNController::train(float target) {
    const uint16_t ext_size = n_reservoir + n_inputs;

    // Добавление шума
    float* noisy_state = new float[ext_size];
    for (uint16_t i = 0; i < ext_size; i++) {
        noisy_state[i] = extended_state[i] + rand_gaussian() * noise_level;
    }

    const float prediction = predict();
    const float error = target - prediction;

    // P · s
    float* P_s = new float[ext_size];
    for (uint16_t i = 0; i < ext_size; i++) {
        float sum = 0;
        const float* P_row = &P[i * ext_size];
        for (uint16_t j = 0; j < ext_size; j++) {
            sum += P_row[j] * noisy_state[j];
        }
        P_s[i] = sum;
    }

    // Знаменатель: λ + s^T · P · s + reg
    float denom = lambda + regularization;
    for (uint16_t i = 0; i < ext_size; i++) {
        denom += noisy_state[i] * P_s[i];
    }
    if (fabsf(denom) < 1e-12f) denom = 1e-12f;

    // Вектор усиления
    float* g = new float[ext_size];
    for (uint16_t i = 0; i < ext_size; i++) {
        g[i] = P_s[i] / denom;
    }

    // Обновление W_out
    for (uint16_t i = 0; i < ext_size; i++) {
        W_out[i] += error * g[i];
    }

    // s^T · P — один раз, обход по строкам
    float* sP = new float[ext_size];
    for (uint16_t j = 0; j < ext_size; j++) {
        sP[j] = 0.0f;
    }
    for (uint16_t k = 0; k < ext_size; k++) {
        const float sk = noisy_state[k];
        const float* P_row = &P[k * ext_size];
        for (uint16_t j = 0; j < ext_size; j++) {
            sP[j] += sk * P_row[j];
        }
    }

    // Обновление P = (P - g · sP) / λ
    float* new_P = new float[ext_size * ext_size];
    for (uint16_t i = 0; i < ext_size; i++) {
        const float gi = g[i];
        const float* P_row = &P[i * ext_size];
        float* newP_row = &new_P[i * ext_size];
        for (uint16_t j = 0; j < ext_size; j++) {
            newP_row[j] = (P_row[j] - gi * sP[j]) / lambda;
        }
    }

    // Симметризация P
    for (uint16_t i = 0; i < ext_size; i++) {
        for (uint16_t j = i + 1; j < ext_size; j++) {
            float avg = (new_P[i * ext_size + j] + new_P[j * ext_size + i]) * 0.5f;
            new_P[i * ext_size + j] = avg;
            new_P[j * ext_size + i] = avg;
        }
    }
    for (uint16_t i = 0; i < ext_size; i++) {
        new_P[i * ext_size + i] += regularization;
    }

    memcpy(P, new_P, ext_size * ext_size * sizeof(float));

    delete[] noisy_state;
    delete[] P_s;
    delete[] g;
    delete[] sP;
    delete[] new_P;
}

// ---------------------------------------------------------------------------
// Сброс состояния сети и RLS, нормализация сбрасывается в начальное
// ---------------------------------------------------------------------------
void ESNController::reset() {
    memset(state, 0, n_reservoir * sizeof(float));
    memset(extended_state, 0, (n_reservoir + n_inputs) * sizeof(float));
    memset(W_out, 0, (n_reservoir + n_inputs) * sizeof(float));
    init_rls();
    for (uint8_t i = 0; i < n_inputs; i++) {
        input_mean[i] = 0.0f;
        input_std[i] = 1.0f;
    }
}

bool ESNController::reload_params()
{
    bool need_full_reinit = false;

    // Размер резервуара — требует полной переаллокации
    if ((uint8_t)cfg_reservoir_size.get() != n_reservoir) {
        need_full_reinit = true;
    }

    // Спектральный радиус — требует пересчёта W_res (для простоты тоже полный re-init)
    if (fabsf(cfg_spectral_radius.get() - spectral_radius) > 1e-4f) {
        need_full_reinit = true;
    }

    // Остальные — просто обновляем члены класса
    leak_rate      = cfg_leak_rate.get();
    noise_level    = cfg_noise_level.get();
    regularization = cfg_regularization.get();
    lambda         = cfg_lambda.get();

    return need_full_reinit;
}


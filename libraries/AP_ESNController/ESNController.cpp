#include "ESNController.h"
#include <AP_Math/AP_Math.h>
#include <cstdlib>
#include <cstdio>
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

// ---------------------------------------------------------------------------
// Освобождение памяти. delete[] на nullptr безопасен — можно вызывать
// многократно, в том числе из деструктора после init().
// ---------------------------------------------------------------------------
void ESNController::free_buffers() {
    delete[] W_in;           W_in = nullptr;
    delete[] W_res;          W_res = nullptr;
    delete[] W_out;          W_out = nullptr;
    delete[] state;          state = nullptr;
    delete[] extended_state; extended_state = nullptr;
    delete[] P;              P = nullptr;
    delete[] input_mean;     input_mean = nullptr;
    delete[] input_std;      input_std = nullptr;

    delete[] _u_norm;            _u_norm = nullptr;
    delete[] _reservoir_scratch; _reservoir_scratch = nullptr;
    delete[] _noisy_state;       _noisy_state = nullptr;
    delete[] _P_s;               _P_s = nullptr;
    delete[] _g;                 _g = nullptr;
    delete[] _sP;                _sP = nullptr;
    delete[] _new_P;             _new_P = nullptr;
    delete[] _eig_vec;           _eig_vec = nullptr;
    delete[] _eig_new_vec;       _eig_new_vec = nullptr;
    delete[] gain;               gain = nullptr;
    delete[] bias;               bias = nullptr;
}

// ---------------------------------------------------------------------------
// Единственное место, где выделяется вся память.
// Вызывается только из init().
// ---------------------------------------------------------------------------
void ESNController::alloc_buffers() {
    const uint16_t ext_size = n_reservoir + n_inputs;

    W_in  = new float[n_reservoir * n_inputs];
    W_res = new float[n_reservoir * n_reservoir];
    W_out = new float[ext_size];

    state          = new float[n_reservoir];
    extended_state = new float[ext_size];
    P              = new float[ext_size * ext_size];
    input_mean     = new float[n_inputs];
    input_std      = new float[n_inputs];

    // Предварительно выделенные рабочие буферы
    _u_norm            = new float[n_inputs];
    _reservoir_scratch = new float[n_reservoir];
    _noisy_state       = new float[ext_size];
    _P_s               = new float[ext_size];
    _g                 = new float[ext_size];
    _sP                = new float[ext_size];
    _new_P             = new float[ext_size * ext_size];
    _eig_vec           = new float[n_reservoir];
    _eig_new_vec       = new float[n_reservoir];
    gain               = new float[n_reservoir];
    bias               = new float[n_reservoir];
}

void ESNController::init(uint8_t _n_inputs, uint8_t _n_reservoir,
                         float _sr, float _lr, float _noise,
                         float _reg, float _lam)
{
    // Освобождаем всё, что было выделено ранее (если init вызывается повторно)
    free_buffers();

    // Читаем из AP_Param, если значение задано; иначе используем переданное
    n_inputs = _n_inputs;
    n_reservoir = (cfg_reservoir_size.get() > 0) ? cfg_reservoir_size.get() : _n_reservoir;
    spectral_radius = (cfg_spectral_radius.get() > 0.0f) ? cfg_spectral_radius.get() : _sr;
    leak_rate       = (cfg_leak_rate.get() > 0.0f)      ? cfg_leak_rate.get()       : _lr;
    noise_level     = (cfg_noise_level.get() >= 0.0f)   ? cfg_noise_level.get()     : _noise;
    regularization  = (cfg_regularization.get() > 0.0f) ? cfg_regularization.get()  : _reg;
    lambda          = (cfg_lambda.get() > 0.0f)         ? cfg_lambda.get()          : _lam;

    alloc_buffers();

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

    for (uint8_t i = 0; i < n_reservoir; i++) {
        gain[i] = 1.0f;   // начальный gain
        bias[i] = 0.0f;   // начальный bias
    }
}

ESNController::~ESNController() {
    free_buffers();
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
    // Используем предаллоцированные _eig_vec / _eig_new_vec — без new/delete в цикле.
    for (uint8_t i = 0; i < n_reservoir; i++) {
        _eig_vec[i] = rand_gaussian();
    }
    float prev_norm = 0;
    float max_eig = 1.0f;
    for (int iter = 0; iter < 50; iter++) {
        for (uint8_t i = 0; i < n_reservoir; i++) {
            float sum = 0;
            for (uint8_t j = 0; j < n_reservoir; j++) {
                sum += W_res[i * n_reservoir + j] * _eig_vec[j];
            }
            _eig_new_vec[i] = sum;
        }
        float norm = 0;
        for (uint8_t i = 0; i < n_reservoir; i++) {
            norm += _eig_new_vec[i] * _eig_new_vec[i];
        }
        norm = sqrtf(norm);
        if (iter > 10 && prev_norm > 0) {
            max_eig = norm / prev_norm;
            if (max_eig < 0) max_eig = -max_eig;
            if (max_eig > 1e-4) break;
        }
        prev_norm = norm;
        for (uint8_t i = 0; i < n_reservoir; i++) {
            _eig_vec[i] = _eig_new_vec[i];
        }
    }

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
// Рекуррентная часть считается от СТАРОГО state в scratch-буфер,
// затем к ней добавляется входная часть и state обновляется на месте.
// ---------------------------------------------------------------------------
void ESNController::compute_new_state(const float* u_norm) {
    // W_res · state (старое состояние)
    for (uint8_t i = 0; i < n_reservoir; i++) {
        float sum = 0;
        for (uint8_t j = 0; j < n_reservoir; j++) {
            sum += W_res[i * n_reservoir + j] * state[j];
        }
        _reservoir_scratch[i] = sum;
    }

    // W_in · u + предыдущий результат → tanh → leaky update
    for (uint8_t i = 0; i < n_reservoir; i++) {
        float sum = _reservoir_scratch[i];
        for (uint8_t j = 0; j < n_inputs; j++) {
            sum += W_in[i * n_inputs + j] * u_norm[j];
        }
        float activation = tanhf(gain[i] * sum + bias[i]);
        state[i] = (1 - leak_rate) * state[i] + leak_rate * activation;
    }
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
    normalize(u, _u_norm);
    compute_new_state(_u_norm);
    form_extended_state(_u_norm);
    return predict();
}

// ---------------------------------------------------------------------------
// RLS-обучение: корректирует W_out по ошибке target - prediction
// ---------------------------------------------------------------------------
void ESNController::train(float target) {
    const uint16_t ext_size = n_reservoir + n_inputs;
    const float inv_lambda = 1.0f / lambda;
    // Добавление шума
    for (uint16_t i = 0; i < ext_size; i++) {
        _noisy_state[i] = extended_state[i] + rand_gaussian() * noise_level;
    }

    const float prediction = predict();
    const float error = target - prediction;

    // P · s
    for (uint16_t i = 0; i < ext_size; i++) {
        float sum = 0;
        const float* P_row = &P[i * ext_size];
        for (uint16_t j = 0; j < ext_size; j++) {
            sum += P_row[j] * _noisy_state[j];
        }
        _P_s[i] = sum;
    }

    // Знаменатель: λ + s^T · P · s + reg
    float denom = lambda + regularization;
    for (uint16_t i = 0; i < ext_size; i++) {
        denom += _noisy_state[i] * _P_s[i];
    }
    if (fabsf(denom) < 1e-12f) denom = 1e-12f;

    // Вектор усиления
    for (uint16_t i = 0; i < ext_size; i++) {
        _g[i] = _P_s[i] / denom;
    }

    // Обновление W_out
    for (uint16_t i = 0; i < ext_size; i++) {
        W_out[i] += error * _g[i];
    }

    // s^T · P — один раз, обход по строкам
    for (uint16_t j = 0; j < ext_size; j++) {
        _sP[j] = 0.0f;
    }
    for (uint16_t k = 0; k < ext_size; k++) {
        const float sk = _noisy_state[k];
        const float* P_row = &P[k * ext_size];
        for (uint16_t j = 0; j < ext_size; j++) {
            _sP[j] += sk * P_row[j];
        }
    }

    // new_P = (P - g · sP) / λ
    for (uint16_t i = 0; i < ext_size; i++) {
        const float gi = _g[i];
        const float* P_row = &P[i * ext_size];
        float* newP_row = &_new_P[i * ext_size];
        for (uint16_t j = 0; j < ext_size; j++) {
            newP_row[j] = (P_row[j] - gi * _sP[j]) * inv_lambda;
        }
    }

    // Симметризация P
    for (uint16_t i = 0; i < ext_size; i++) {
        for (uint16_t j = i + 1; j < ext_size; j++) {
            float avg = (_new_P[i * ext_size + j] + _new_P[j * ext_size + i]) * 0.5f;
            _new_P[i * ext_size + j] = avg;
            _new_P[j * ext_size + i] = avg;
        }
    }
    for (uint16_t i = 0; i < ext_size; i++) {
        _new_P[i * ext_size + i] += regularization;
    }

    memcpy(P, _new_P, ext_size * ext_size * sizeof(float));
}

// ---------------------------------------------------------------------------
// Предобучение сети
// ---------------------------------------------------------------------------
void ESNController::pretrain(const float* data, uint16_t num_samples, 
                             float eta, float mu, float sigma, uint8_t epochs)
{
    if (data == nullptr || num_samples == 0) {
        return;
    }

    // Буфер для нормализованных входов (можно использовать _u_norm, но лучше локальный)
    float* u_norm = new float[n_inputs];

    for (uint8_t epoch = 0; epoch < epochs; epoch++) {
        // Сбрасываем состояние резервуара перед каждой эпохой
        memset(state, 0, n_reservoir * sizeof(float));

        for (uint16_t s = 0; s < num_samples; s++) {
            const float* u = &data[s * n_inputs];

            // 1. Нормализация входа (используем существующую функцию)
            normalize(u, u_norm);

            // 2. Вычисление нового состояния с текущими gain/bias
            //    (аналогично compute_new_state, но без обновления state до конца)
            float* input_part = new float[n_reservoir];
            float* reservoir_part = new float[n_reservoir];
            for (uint8_t i = 0; i < n_reservoir; i++) {
                float sum_in = 0;
                for (uint8_t j = 0; j < n_inputs; j++) {
                    sum_in += W_in[i * n_inputs + j] * u_norm[j];
                }
                float sum_res = 0;
                for (uint8_t j = 0; j < n_reservoir; j++) {
                    sum_res += W_res[i * n_reservoir + j] * state[j];
                }
                input_part[i] = sum_in;
                reservoir_part[i] = sum_res;
            }

            // 3. IP-обновление gain/bias для каждого нейрона
            for (uint8_t i = 0; i < n_reservoir; i++) {
                // Взвешенная сумма до активации
                float x = input_part[i] + reservoir_part[i];
                // Выход нейрона (tanh)
                float y = tanhf(gain[i] * x + bias[i]);

                // Формулы для tanh (гауссово распределение)
                // Δb = -η * [ -μ/σ² + (y/σ²)*(2σ² + 1 - y² + μ*y) ]
                // Δa = η/a + Δb * x
                float inv_sigma2 = 1.0f / (sigma * sigma);
                float term = -mu * inv_sigma2 + (y * inv_sigma2) * (2.0f * sigma * sigma + 1.0f - y * y + mu * y);
                float delta_b = -eta * term;
                float delta_a = eta / gain[i] + delta_b * x;

                // Обновляем параметры
                bias[i] += delta_b;
                gain[i] += delta_a;

                // Ограничиваем gain, чтобы избежать численных проблем
                gain[i] = constrain_float(gain[i], 0.1f, 5.0f);

                // 4. Обновляем состояние резервуара с новым gain/bias
                float new_activation = tanhf(gain[i] * x + bias[i]);
                state[i] = (1.0f - leak_rate) * state[i] + leak_rate * new_activation;
            }

            delete[] input_part;
            delete[] reservoir_part;
        }
    }

    delete[] u_norm;

    // После предобучения сбрасываем выходные веса и RLS,
    // так как динамика резервуара изменилась.
    memset(W_out, 0, (n_reservoir + n_inputs) * sizeof(float));
    init_rls();

    // Сбрасываем состояние, чтобы начать с чистого листа
    memset(state, 0, n_reservoir * sizeof(float));
    memset(extended_state, 0, (n_reservoir + n_inputs) * sizeof(float));
}

// ---------------------------------------------------------------------------
// Считывание входных данных для предобучения сети
// ---------------------------------------------------------------------------
uint16_t ESNController::load_pretrain_data(const char* filename, float** out_data)
{
    FILE* f = fopen(filename, "r");
    if (!f) {
        return 0;
    }

    char line[512];
    // Пропускаем заголовок
    if (!fgets(line, sizeof(line), f)) {
        fclose(f);
        return 0;
    }

    // Первый проход: считаем строки
    uint16_t count = 0;
    while (fgets(line, sizeof(line), f)) {
        count++;
    }
    rewind(f);
    if (fgets(line, sizeof(line), f) == nullptr) {
        fclose(f);
        return 0;
    }

    // Выделяем память под данные: count * n_inputs
    float* data = new float[count * n_inputs];
    uint16_t idx = 0;

    while (fgets(line, sizeof(line), f) && idx < count) {
        // Формат: time_us, f0, f1, ...
        char* token = strtok(line, ",");
        if (!token) continue;
        // Пропускаем time_us
        token = strtok(NULL, ",");
        uint8_t fidx = 0;
        while (token && fidx < n_inputs) {
            data[idx * n_inputs + fidx] = atof(token);
            token = strtok(NULL, ",");
            fidx++;
        }
        // Если признаков меньше n_inputs, дополняем нулями
        while (fidx < n_inputs) {
            data[idx * n_inputs + fidx++] = 0.0f;
        }
        idx++;
    }

    fclose(f);
    *out_data = data;
    return idx;
}

// ---------------------------------------------------------------------------
// Сброс состояния сети и RLS
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
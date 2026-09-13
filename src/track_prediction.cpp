#include "robotnav/perception/track_prediction.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace robotnav::perception {
namespace {

constexpr double kNanosecondsPerSecond = 1e9;
constexpr double kTwoPi = 6.28318530717958647692;
using Vector4 = std::array<double, 4>;
using Matrix4 = std::array<std::array<double, 4>, 4>;

struct CvModel {
    Vector4 state{};
    Matrix4 covariance{};
    TimestampNs timestamp_ns = 0;
};

bool positiveFinite(double value) {
    return std::isfinite(value) && value > 0.0;
}

CvModel initializeModel(const ObstacleTrack& track,
                        double initial_velocity_variance,
                        double default_measurement_variance) {
    CvModel model;
    model.state = {track.position.x, track.position.y,
                   track.velocity.x, track.velocity.y};
    model.timestamp_ns = track.timestamp_ns;
    const double xx = track.position_covariance.xx > 0.0
        ? track.position_covariance.xx : default_measurement_variance;
    const double yy = track.position_covariance.yy > 0.0
        ? track.position_covariance.yy : default_measurement_variance;
    model.covariance[0][0] = xx;
    model.covariance[0][1] = track.position_covariance.xy;
    model.covariance[1][0] = track.position_covariance.xy;
    model.covariance[1][1] = yy;
    model.covariance[2][2] = initial_velocity_variance;
    model.covariance[3][3] = initial_velocity_variance;
    return model;
}

void predictModel(CvModel& model, TimestampNs timestamp_ns,
                  double acceleration_variance) {
    const double dt = static_cast<double>(timestamp_ns - model.timestamp_ns) /
        kNanosecondsPerSecond;
    if (dt == 0.0) return;
    Matrix4 transition{};
    for (int index = 0; index < 4; ++index) transition[index][index] = 1.0;
    transition[0][2] = dt;
    transition[1][3] = dt;

    Matrix4 intermediate{};
    Matrix4 predicted_covariance{};
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            for (int inner = 0; inner < 4; ++inner) {
                intermediate[row][column] +=
                    transition[row][inner] * model.covariance[inner][column];
            }
        }
    }
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            for (int inner = 0; inner < 4; ++inner) {
                predicted_covariance[row][column] +=
                    intermediate[row][inner] * transition[column][inner];
            }
        }
    }

    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double q = acceleration_variance;
    predicted_covariance[0][0] += q * dt4 / 4.0;
    predicted_covariance[1][1] += q * dt4 / 4.0;
    predicted_covariance[0][2] += q * dt3 / 2.0;
    predicted_covariance[2][0] += q * dt3 / 2.0;
    predicted_covariance[1][3] += q * dt3 / 2.0;
    predicted_covariance[3][1] += q * dt3 / 2.0;
    predicted_covariance[2][2] += q * dt2;
    predicted_covariance[3][3] += q * dt2;

    model.state[0] += model.state[2] * dt;
    model.state[1] += model.state[3] * dt;
    model.covariance = predicted_covariance;
    model.timestamp_ns = timestamp_ns;
}

Covariance2d measurementCovariance(const ObstacleTrack& observation,
                                    double default_variance) {
    auto covariance = observation.position_covariance;
    if (covariance.xx == 0.0 && covariance.yy == 0.0) {
        covariance.xx = default_variance;
        covariance.yy = default_variance;
    }
    return covariance;
}

double correctModel(CvModel& model, const ObstacleTrack& observation,
                    double default_measurement_variance) {
    const auto measurement_covariance = measurementCovariance(
        observation, default_measurement_variance);
    const double innovation_x = observation.position.x - model.state[0];
    const double innovation_y = observation.position.y - model.state[1];
    const double s00 = model.covariance[0][0] + measurement_covariance.xx;
    const double s01 = model.covariance[0][1] + measurement_covariance.xy;
    const double s11 = model.covariance[1][1] + measurement_covariance.yy;
    const double determinant = s00 * s11 - s01 * s01;
    if (!std::isfinite(determinant) || determinant <= 1e-15) return 0.0;
    const double inverse00 = s11 / determinant;
    const double inverse01 = -s01 / determinant;
    const double inverse11 = s00 / determinant;

    std::array<std::array<double, 2>, 4> gain{};
    for (int row = 0; row < 4; ++row) {
        gain[row][0] = model.covariance[row][0] * inverse00 +
            model.covariance[row][1] * inverse01;
        gain[row][1] = model.covariance[row][0] * inverse01 +
            model.covariance[row][1] * inverse11;
        model.state[row] += gain[row][0] * innovation_x +
            gain[row][1] * innovation_y;
    }

    const Matrix4 prior = model.covariance;
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            model.covariance[row][column] = prior[row][column] -
                gain[row][0] * prior[0][column] -
                gain[row][1] * prior[1][column];
        }
    }
    for (int row = 0; row < 4; ++row) {
        for (int column = row + 1; column < 4; ++column) {
            const double symmetric = 0.5 *
                (model.covariance[row][column] +
                 model.covariance[column][row]);
            model.covariance[row][column] = symmetric;
            model.covariance[column][row] = symmetric;
        }
        model.covariance[row][row] =
            std::max(0.0, model.covariance[row][row]);
    }

    const double mahalanobis = innovation_x *
        (inverse00 * innovation_x + inverse01 * innovation_y) +
        innovation_y *
        (inverse01 * innovation_x + inverse11 * innovation_y);
    return std::exp(-0.5 * std::max(0.0, mahalanobis)) /
        std::sqrt(kTwoPi * kTwoPi * determinant);
}

PredictedTrackState toPrediction(const CvModel& model) {
    PredictedTrackState result;
    result.timestamp_ns = model.timestamp_ns;
    result.position = {model.state[0], model.state[1]};
    result.velocity = {model.state[2], model.state[3]};
    result.position_covariance = {
        model.covariance[0][0], model.covariance[0][1],
        model.covariance[1][1]};
    return result;
}

CvModel mixModels(const std::array<CvModel, 2>& models,
                  const std::array<double, 2>& weights,
                  TimestampNs timestamp_ns) {
    CvModel mixed;
    mixed.timestamp_ns = timestamp_ns;
    for (int model = 0; model < 2; ++model) {
        for (int row = 0; row < 4; ++row) {
            mixed.state[row] += weights[model] * models[model].state[row];
        }
    }
    for (int model = 0; model < 2; ++model) {
        for (int row = 0; row < 4; ++row) {
            for (int column = 0; column < 4; ++column) {
                const double row_delta =
                    models[model].state[row] - mixed.state[row];
                const double column_delta =
                    models[model].state[column] - mixed.state[column];
                mixed.covariance[row][column] += weights[model] *
                    (models[model].covariance[row][column] +
                     row_delta * column_delta);
            }
        }
    }
    return mixed;
}

PredictedTrackState combineModels(
    const std::array<CvModel, 2>& models,
    const std::array<double, 2>& probabilities) {
    const auto combined = mixModels(
        models, probabilities, models.front().timestamp_ns);
    auto result = toPrediction(combined);
    result.smooth_model_probability = probabilities[0];
    result.maneuver_model_probability = probabilities[1];
    return result;
}

PredictionStatus validateObservation(const ObstacleTrack& observation) {
    return validate(observation)
        ? PredictionStatus::Success : PredictionStatus::InvalidTrack;
}

}  // namespace

struct KalmanTrackPredictor::Impl {
    explicit Impl(KalmanPredictorOptions input_options)
        : options(std::move(input_options)) {
        if (!positiveFinite(options.process_acceleration_variance) ||
            !positiveFinite(options.default_measurement_variance) ||
            !positiveFinite(options.initial_velocity_variance)) {
            throw std::invalid_argument("invalid Kalman predictor options");
        }
    }

    KalmanPredictorOptions options;
    CvModel model;
    bool initialized = false;
};

KalmanTrackPredictor::KalmanTrackPredictor(KalmanPredictorOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
KalmanTrackPredictor::~KalmanTrackPredictor() = default;
KalmanTrackPredictor::KalmanTrackPredictor(
    KalmanTrackPredictor&&) noexcept = default;
KalmanTrackPredictor& KalmanTrackPredictor::operator=(
    KalmanTrackPredictor&&) noexcept = default;

PredictionStatus KalmanTrackPredictor::update(
    const ObstacleTrack& observation) {
    const auto validation = validateObservation(observation);
    if (validation != PredictionStatus::Success) return validation;
    if (!impl_->initialized) {
        impl_->model = initializeModel(
            observation, impl_->options.initial_velocity_variance,
            impl_->options.default_measurement_variance);
        impl_->initialized = true;
        return PredictionStatus::Success;
    }
    if (observation.timestamp_ns <= impl_->model.timestamp_ns) {
        return PredictionStatus::NonMonotonicTimestamp;
    }
    predictModel(impl_->model, observation.timestamp_ns,
                 impl_->options.process_acceleration_variance);
    correctModel(impl_->model, observation,
                 impl_->options.default_measurement_variance);
    return PredictionStatus::Success;
}

TrackPredictionResult KalmanTrackPredictor::predict(
    TimestampNs timestamp_ns) const {
    if (!impl_->initialized) {
        return {PredictionStatus::NotInitialized, {}};
    }
    if (timestamp_ns < impl_->model.timestamp_ns) {
        return {PredictionStatus::NonMonotonicTimestamp, {}};
    }
    auto predicted = impl_->model;
    predictModel(predicted, timestamp_ns,
                 impl_->options.process_acceleration_variance);
    return {PredictionStatus::Success, toPrediction(predicted)};
}

void KalmanTrackPredictor::reset() noexcept {
    impl_->model = {};
    impl_->initialized = false;
}

bool KalmanTrackPredictor::initialized() const noexcept {
    return impl_->initialized;
}

struct ImmTrackPredictor::Impl {
    explicit Impl(ImmPredictorOptions input_options)
        : options(std::move(input_options)) {
        if (!positiveFinite(options.smooth_process_acceleration_variance) ||
            !positiveFinite(options.maneuver_process_acceleration_variance) ||
            options.maneuver_process_acceleration_variance <=
                options.smooth_process_acceleration_variance ||
            !positiveFinite(options.default_measurement_variance) ||
            !positiveFinite(options.initial_velocity_variance) ||
            !std::isfinite(options.initial_maneuver_probability) ||
            options.initial_maneuver_probability < 0.0 ||
            options.initial_maneuver_probability > 1.0 ||
            !std::isfinite(options.smooth_stay_probability) ||
            options.smooth_stay_probability <= 0.0 ||
            options.smooth_stay_probability >= 1.0 ||
            !std::isfinite(options.maneuver_stay_probability) ||
            options.maneuver_stay_probability <= 0.0 ||
            options.maneuver_stay_probability >= 1.0) {
            throw std::invalid_argument("invalid IMM predictor options");
        }
        probabilities = {1.0 - options.initial_maneuver_probability,
                         options.initial_maneuver_probability};
    }

    ImmPredictorOptions options;
    std::array<CvModel, 2> models;
    std::array<double, 2> probabilities;
    bool initialized = false;
};

ImmTrackPredictor::ImmTrackPredictor(ImmPredictorOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
ImmTrackPredictor::~ImmTrackPredictor() = default;
ImmTrackPredictor::ImmTrackPredictor(ImmTrackPredictor&&) noexcept = default;
ImmTrackPredictor& ImmTrackPredictor::operator=(
    ImmTrackPredictor&&) noexcept = default;

PredictionStatus ImmTrackPredictor::update(
    const ObstacleTrack& observation) {
    const auto validation = validateObservation(observation);
    if (validation != PredictionStatus::Success) return validation;
    if (!impl_->initialized) {
        const auto initial = initializeModel(
            observation, impl_->options.initial_velocity_variance,
            impl_->options.default_measurement_variance);
        impl_->models = {initial, initial};
        impl_->initialized = true;
        return PredictionStatus::Success;
    }
    if (observation.timestamp_ns <= impl_->models[0].timestamp_ns) {
        return PredictionStatus::NonMonotonicTimestamp;
    }

    const double transition[2][2] = {
        {impl_->options.smooth_stay_probability,
         1.0 - impl_->options.smooth_stay_probability},
        {1.0 - impl_->options.maneuver_stay_probability,
         impl_->options.maneuver_stay_probability}};
    std::array<double, 2> destination_probability{};
    std::array<CvModel, 2> mixed_models;
    for (int destination = 0; destination < 2; ++destination) {
        for (int source = 0; source < 2; ++source) {
            destination_probability[destination] +=
                impl_->probabilities[source] * transition[source][destination];
        }
        std::array<double, 2> mixing_weights{};
        for (int source = 0; source < 2; ++source) {
            mixing_weights[source] = impl_->probabilities[source] *
                transition[source][destination] /
                destination_probability[destination];
        }
        mixed_models[destination] = mixModels(
            impl_->models, mixing_weights,
            impl_->models.front().timestamp_ns);
    }

    const double process_variance[2] = {
        impl_->options.smooth_process_acceleration_variance,
        impl_->options.maneuver_process_acceleration_variance};
    std::array<double, 2> likelihood{};
    for (int model = 0; model < 2; ++model) {
        predictModel(mixed_models[model], observation.timestamp_ns,
                     process_variance[model]);
        likelihood[model] = correctModel(
            mixed_models[model], observation,
            impl_->options.default_measurement_variance);
    }
    const double normalization =
        destination_probability[0] * likelihood[0] +
        destination_probability[1] * likelihood[1];
    if (normalization > std::numeric_limits<double>::min()) {
        for (int model = 0; model < 2; ++model) {
            impl_->probabilities[model] =
                destination_probability[model] * likelihood[model] /
                normalization;
        }
    } else {
        impl_->probabilities = destination_probability;
    }
    impl_->models = mixed_models;
    return PredictionStatus::Success;
}

TrackPredictionResult ImmTrackPredictor::predict(
    TimestampNs timestamp_ns) const {
    if (!impl_->initialized) {
        return {PredictionStatus::NotInitialized, {}};
    }
    if (timestamp_ns < impl_->models[0].timestamp_ns) {
        return {PredictionStatus::NonMonotonicTimestamp, {}};
    }
    auto predicted = impl_->models;
    predictModel(predicted[0], timestamp_ns,
                 impl_->options.smooth_process_acceleration_variance);
    predictModel(predicted[1], timestamp_ns,
                 impl_->options.maneuver_process_acceleration_variance);
    return {PredictionStatus::Success,
            combineModels(predicted, impl_->probabilities)};
}

void ImmTrackPredictor::reset() noexcept {
    impl_->models = {};
    impl_->probabilities = {
        1.0 - impl_->options.initial_maneuver_probability,
        impl_->options.initial_maneuver_probability};
    impl_->initialized = false;
}

bool ImmTrackPredictor::initialized() const noexcept {
    return impl_->initialized;
}

}  // namespace robotnav::perception

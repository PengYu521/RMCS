#include "rmcs_core/controller/component.h"
#include <cmath>

class ComponentA : public rmcs_core::controller::Component {
public:
    ComponentA() : Component("component_a", 1000) { // 1000Hz
        declareOutput("sin_omega_t", 0.0);
        declareOutput("cos_omega_t", 0.0);
        declareParameter("omega", 1.0);
    }

    void onInit() override {
        getParameter("omega", omega_);
        time_ = 0.0;
        period_ = 1.0 / getFrequency();
    }

    void update() override {
        double sin_val = std::sin(omega_ * time_);
        double cos_val = std::cos(omega_ * time_);
        setOutput("sin_omega_t", sin_val);
        setOutput("cos_omega_t", cos_val);
        time_ += period_;
    }

private:
    double omega_;
    double time_;
    double period_;
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::sin_cos_module::ComponentA, rmcs_executor::Component)
RMCS_REGISTER_COMPONENT(ComponentA, "sin_cos_module")
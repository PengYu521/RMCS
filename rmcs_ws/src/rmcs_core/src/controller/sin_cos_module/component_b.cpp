#include "rmcs_core/controller/component.h"

class ComponentB : public rmcs_core::controller::Component {
public:
    ComponentB() : Component("component_b", 1000) {
        declareInput("sin_omega_t", 0.0);
        declareInput("cos_omega_t", 0.0);
        declareOutput("sum_sin_cos", 0.0);
    }

    void update() override {
        double sin_val = getInput("sin_omega_t");
        double cos_val = getInput("cos_omega_t");
        setOutput("sum_sin_cos", sin_val + cos_val);
    }
};

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::sin_cos_module::ComponentB, rmcs_executor::Component)

RMCS_REGISTER_COMPONENT(ComponentB, "sin_cos_module")
#pragma once

#include <esphome/core/component.h>
#include <esphome/components/text_sensor/text_sensor.h>
#include <string>

namespace esphome {
namespace opnpool {

class OpnPool;  // Forward declaration only - don't include opnpool.h!

class OpnPoolTextSensor : public text_sensor::TextSensor, public Component {
  public:
    void setup() override;
    void dump_config() override;
    
    void set_parent(OpnPool *parent) { this->parent_ = parent; }
    
    void publish_state_if_changed(const std::string &state);


  protected:
    OpnPool *parent_{nullptr};
    std::string last_state_{};
    bool last_state_valid_{false};
};

}  // namespace opnpool
}  // namespace esphome
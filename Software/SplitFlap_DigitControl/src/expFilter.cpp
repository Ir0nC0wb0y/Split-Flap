#include <expFilter.h>

expFilter::expFilter() {
    return;
}

void expFilter::setWeight(float weight) {
  if (weight > 1) {
    weight = 1;
  } else if (weight < 0) {
    weight = 0;
  }
    _weight = weight;
}

float expFilter::filter(float measurement) {
  _value = _weight*_value + (1-_weight)*measurement;
  return _value;
}

void expFilter::setValue(float value) {
  _value = value;
}

float expFilter::getValue() {
    return _value;
}

float expFilter::getWeight() {
    return _weight;
}
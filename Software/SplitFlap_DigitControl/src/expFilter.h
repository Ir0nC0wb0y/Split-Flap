// Exponential Filter
class expFilter {
    public:
      expFilter();
      void setWeight(float weight);
      float filter(float measurement);
      void setValue(float value);
      float getValue();
      float getWeight();

    private:
      float _weight = 0.0;
      float _value = 0.0;
  };
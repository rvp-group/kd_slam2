#pragma once

namespace kd_slam {
  namespace utils {

    template <typename T>
    inline T ZeroConstantValue_() {return T::Zero();}

    template <>
    inline int ZeroConstantValue_<int>() {return 0;}

    template <>
    inline size_t ZeroConstantValue_<size_t>() {return 0;}

    template <>
    inline float ZeroConstantValue_<float>() {return 0.;}

    template <>
    inline double ZeroConstantValue_<double>() {return 0.;}

    template <typename T, int BATCH_SIZE_=1024, int MAX_LEVELS_=5>
    struct StableAdder_ {
      static constexpr int MAX_LEVELS = MAX_LEVELS_;
      static constexpr int BATCH_SIZE = BATCH_SIZE_;
      size_t count=0;
      struct AccumulatorEntry {
        T value;
        size_t count=0;
        inline void add(const T& v) {
          value+=v;
          ++count;
        }
        inline void reset() {
          value=ZeroConstantValue_<T>();
          count=0;
        }
      };

      AccumulatorEntry _accumulators[MAX_LEVELS];

      inline void reset() {
        for (int i=0; i<MAX_LEVELS; ++i) {
          _accumulators[i].reset();
        }
        count=0;
      }

      StableAdder_(){
        reset();
      }

      inline void add(T v) {
        using namespace std;
        ++count;
        int i=0;
        while (i<MAX_LEVELS) {
          _accumulators[i].add(v);
          if (_accumulators[i].count<BATCH_SIZE)
            break;
          v=_accumulators[i].value;
          _accumulators[i].reset();
          ++i;
        }
      }
  
      inline T sum() const {
        using namespace std;
        T acc=ZeroConstantValue_<T>();
        for (int i=0; i<MAX_LEVELS; ++i) {
          acc+=_accumulators[i].value;
        }
        return acc;
      }
    };
  }
}

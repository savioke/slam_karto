#include <open_karto/Karto.h>

namespace karto
{
  class LocalizedRangeScanStamped : public karto::LocalizedRangeScan
  {
    public:
      LocalizedRangeScanStamped(const Name& rSensorName, const RangeReadingsVector &rReadings, double ts) : LocalizedRangeScan(rSensorName, rReadings)
      {
        ts_ = ts;
      }

      double getTimestamp()
      {
        return ts_;
      }

    private:
      double ts_;

  };
}

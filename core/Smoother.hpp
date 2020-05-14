#include "Smoother.h"

namespace L3
{
  template <typename T>
    void LogisticSmoother<T>::smooth(L3::Histogram<T>* hist)  {
      CHECK_NOTNULL(hist);
      double tmp = 200;
      double* bin_ptr = hist->hist->bin;
      for(unsigned int x = 0; x < hist->hist->nx*hist->hist->ny; x++)
      {
        *bin_ptr = std::min(*bin_ptr, tmp);
        bin_ptr++;
      }
    };

  template <typename T, int N>
    void Smoother<T,N>::smooth(L3::Histogram<T>* hist)  {
      CHECK_NOTNULL(hist);
      T* result = new T[hist->x_bins*hist->y_bins];
      CHECK_NOTNULL(result);
      double filter_element=0.0, hist_value=0.0;

      unsigned int query_x, query_y;

#pragma omp parallel shared(hist)
      for(unsigned int x = 0; x < hist->x_bins; x++) {
#pragma omp for nowait
        for(unsigned int y = 0; y < hist->y_bins; y++) {
          hist_value = *(hist->hist->bin + (x*hist->x_bins + y));
          double sum = 0.0;
          for (int filter_index_x = -1*half_step; filter_index_x <= half_step; filter_index_x++) {
            for (int filter_index_y = -1*half_step; filter_index_y <= half_step; filter_index_y++) {
              query_x = x+filter_index_x;
              query_y = y+filter_index_y;

              hist_value = (query_x < hist->x_bins) &&
                (query_y < hist->y_bins) ? gsl_histogram2d_get(hist->hist, query_x, query_y) : 0;

              filter_element = this->filter[filter_index_x+half_step][filter_index_y+half_step];

              sum += hist_value*filter_element;
            }
          }
          result [(x * hist->y_bins) + y] = sum;
        }
      }

      std::copy(result,
                result+(hist->x_bins*hist->y_bins),
                hist->hist->bin);

      delete [] result;
    }
}

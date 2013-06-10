// Smoothing
#include "itpp/signal/filter.h"
#include "itpp/signal/freq_filt.h"

int main()
{
    itpp::vec b;

    itpp::Freq_Filt<double> FF(b,8000);

}

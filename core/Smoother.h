#ifndef L3_SMOOTHER_H
#define L3_SMOOTHER_H

#include "Histogram.h"

namespace L3
{

    template <bool> 
        struct StaticCheck;
    template <>
        struct StaticCheck<true>{};

    template <typename T, int N>
        struct Smoother : StaticCheck<((N%2) != 0) >
        {
            Smoother()
            {
                //if ( (N % 2) == 0 )
                    //throw std::exception();

                half_step = floor(N/2.0);
            }

            double filter[N][N];
            int half_step;

            void smooth( L3::Histogram<T>* hist );

            virtual ~Smoother()
            {

            }

        };

    template <typename T, int N>
        struct BoxSmoother : Smoother<T,N>
        {
            BoxSmoother()
            {

                for( int i=0; i<N; i++ )
                    for( int j=0; j<N; j++ )
                        //this->filter[i][j] = 0.0f;
                        this->filter[i][j] = 0.111f;
          
            }

            void smooth( L3::Histogram<T>* hist )
            {
                Smoother<T,N>::smooth(hist);
            }
            
        };

    template <typename T, int N>
        struct GaussianSmoother : Smoother<T,N>
        {

        };

}
#endif

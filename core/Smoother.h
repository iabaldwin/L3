#ifndef L3_SMOOTHER_H
#define L3_SMOOTHER_H

#include "Histogram.h"

namespace L3
{

    template <bool> 
        struct StaticCheck;
    template <>
        struct StaticCheck<true>{};

    struct SmootherInterface
    {
        virtual ~SmootherInterface()
        {
        }
    };

    template <typename T, int N>
        struct Smoother : StaticCheck<( (N%2) != 0) >, SmootherInterface
        {
            Smoother()
            {
                half_step = floor(N/2.0);
            }

            int half_step;
            double filter[N][N];

            void smooth( L3::Histogram<T>* hist );

            virtual ~Smoother()
            {}

        };

    template <typename T, int N>
        struct BoxSmoother : Smoother<T,N>
        {
            BoxSmoother()
            {
                for( int i=0; i<N; i++ )
                    for( int j=0; j<N; j++ )
                        this->filter[i][j] = 0.111f;
            }

            void smooth( L3::Histogram<T>* hist )
            {
                Smoother<T,N>::smooth(hist);
            }
            
        };

    template <typename T>
        struct GaussianSmoother : Smoother<T,5>
        {
            GaussianSmoother()
            {
                this->filter[0][0]= 1.0/273.0;
                this->filter[0][1]= 4.0/273.0;
                this->filter[0][2]= 7.0/273.0;
                this->filter[0][3]= 4.0/273.0;
                this->filter[0][4]= 1.0/273.0;

                this->filter[1][0]= 4.0/273.0;
                this->filter[1][1]= 16.0/273.0;
                this->filter[1][2]= 26.0/273.0;
                this->filter[1][3]= 16.0/273.0;
                this->filter[1][4]= 4.0/273.0;

                this->filter[2][0]= 7.0/273.0;
                this->filter[2][1]= 26.0/273.0;
                this->filter[2][2]= 41.0/273.0;
                this->filter[2][3]= 26.0/273.0;
                this->filter[2][4]= 7.0/273.0;
                
                this->filter[3][0]= 4.0/273.0;
                this->filter[3][1]= 16.0/273.0;
                this->filter[3][2]= 26.0/273.0;
                this->filter[3][3]= 16.0/273.0;
                this->filter[3][4]= 4.0/273.0;

                this->filter[4][0]= 1.0/273.0;
                this->filter[4][1]= 4.0/273.0;
                this->filter[4][2]= 7.0/273.0;
                this->filter[4][3]= 4.0/273.0;
                this->filter[4][4]= 1.0/273.0;
            }

            void smooth( L3::Histogram<T>* hist )
            {
                Smoother<T,5>::smooth(hist);
            }
            
        };

}
#endif

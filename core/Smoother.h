#ifndef L3_SMOOTHER_H
#define L3_SMOOTHER_H

namespace L3
{

template <typename T, int N>
struct Smoother
{
    float filter[N][N];

    virtual void smooth( L3::Histogram<T>* hist ) = 0;
    
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
                this->filter[i][j] = 0.111f;
    }

    
    void smooth( L3::Histogram<T>* hist ) 
    {

        T* array = new T[ hist->x_bins*hist->y_bins ];

        delete [] array;
    }




};

template <typename T, int N>
struct GaussianSmoother : Smoother<T,N>
{

    void smooth( L3::Histogram<T>* hist ) 
    {

        T* array = new T[ hist->x_bins*hist->y_bins ];

        delete [] array;
    }




};

}
#endif

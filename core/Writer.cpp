#include "Writer.h"

namespace L3
{
    namespace IO
    {

        L3::IO::Writer& operator<<( L3::IO::Writer& o, const std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses )
        {
            //if ( !o.stream.good() )
                //throw std::exception();

            //for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::const_iterator it=poses.begin(); it!=poses.end(); it++ )
            //{
                //o.stream << *(it->second) << std::endl;
            //}

            return o;
        }

        template <>
            size_t BinaryWriter<L3::SE3>::write( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >& data ) 
            {

                size_t written = 0;
                for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = data.begin();
                        it != data.end();
                        it++ )
                {

                    double data = it->first;
                    this->stream.write( (char*)( &data ), sizeof(double) );

                    L3::SE3 pose = (*it->second );
                  
                    data = pose.X();
                    this->stream.write( (char*)( &data ), sizeof(double) );
                    
                    data = pose.Y();
                    this->stream.write( (char*)( &data ), sizeof(double) );
                    
                    data = pose.Z();
                    this->stream.write( (char*)( &data ), sizeof(double) );
                    
                    data = pose.R();
                    this->stream.write( (char*)( &data ), sizeof(double) );
                    
                    data = pose.P();
                    this->stream.write( (char*)( &data ), sizeof(double) );

                    data = pose.Q();
                    this->stream.write( (char*)( &data ), sizeof(double) );

                    written += 7*sizeof(double);
                }

                return written;
            }
    
    
        template <>
            size_t BinaryWriter<L3::LHLV>::write( std::vector< std::pair< double, boost::shared_ptr<L3::LHLV> > >& data ) 
            {

                size_t written = 0;
                for( std::vector< std::pair< double, boost::shared_ptr<L3::LHLV> > >::iterator it = data.begin();
                        it != data.end();
                        it++ )
                {

                    double data = it->first;
                    this->stream.write( (char*)( &data ), sizeof(double) );

                    written += sizeof(double);

                    this->stream.write( (char*)( &(*it->second).data[0]), sizeof(double)*(it->second->data.size() ) );
                    written += sizeof(double)*(it->second->data.size() );
                }            
            
            
            }
    }
}

template size_t L3::IO::BinaryWriter<L3::SE3>::write(std::vector<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > >&);

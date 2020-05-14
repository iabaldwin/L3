#include <iostream>
#include "Reader.h"

int main(int argc, char* argv[])
{
  if (argc <= 1) {
    std::cerr << "Usage: " << argv[0] << " path/to/lidar" << std::endl;
    exit( EXIT_FAILURE );
  }

  boost::shared_ptr<L3::IO::SequentialBinaryReader<L3::LMS151> > reader;
  reader.reset(new L3::IO::SequentialBinaryReader<L3::LMS151>());
  if (not reader->open(std::string{argv[1]})) {
    std::cerr << "No LIDAR available" << std::endl;
    throw std::exception();
  }

  int num_scans{0};
  while(std::size_t read = reader->read()) {
    std::cout << "Read: " << read << " bytes" << std::endl;
    ++num_scans;
    std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > >  scans;
    if ( not reader->extract( scans ) ) {
      std::cerr << "Failed to extract scan!" << std::endl;
      exit(EXIT_FAILURE);
    }
    std::cout << std::fixed << std::setprecision(20) << std::endl;
    std::cout << scans.size() << std::endl;
    std::cout << scans.at(0).first << std::endl;
    std::cout << *scans.at(0).second << std::endl;
    std::cout << "-------"  << std::endl;
  }

  LOG(INFO) << "Done after reading [" << num_scans << "] scans";
  return EXIT_SUCCESS;
}

#include "SpatialData.h"

namespace L3
{
  template <typename T>
    T norm(std::pair<T,T> a, std::pair<T,T> b) {
      return sqrt(pow(a.first-b.first,2) + pow(a.second - b.second, 2));
    }

  /*
   *  Selection Policies
   */
  bool RetrospectiveWithLookaheadPolicy::operator()(std::deque< spatial_data>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window) {
    /*
     *  Initialise distances
     */
    std::vector< std::pair< double, unsigned int > > distances;

    /*
     *  Calculate distances to all sections
     */
    for(unsigned int i=0; i<sections->size(); i++) {
      distances.push_back(std::make_pair(norm(std::make_pair(x, y), std::make_pair((*sections)[i].x, (*sections)[i].y) ), i));
    }

    /*
     *  Sort the distances
     */
    std::sort(distances.begin(), distances.end());

    required_sections.clear();

    int window_cpy(window);

    int lookahead = 0;

    int start_val = (distances.front().second + lookahead) % sections->size();

    for(int i=start_val; (window_cpy)-->0; i--) {
      int load_val = i >=0 ? i : sections->size()-i;
      required_sections.push_front(load_val);
    }
    return true;
  }

  bool StrictlyRetrospectivePolicy::operator()(std::deque< spatial_data>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window) {
    /*
     *  Initialise distances
     */
    std::vector< std::pair< double, unsigned int > > distances;

    /*
     *  Calculate distances to all sections
     */
    for(unsigned int i=0; i<sections->size(); i++) {
      distances.push_back(std::make_pair(norm(std::make_pair(x, y), std::make_pair((*sections)[i].x, (*sections)[i].y) ), i));
    }

    /*
     *  Sort the distances
     */
    std::sort(distances.begin(), distances.end());

    required_sections.clear();

    int window_cpy(window);

    //Here is the magic
    for(int i=distances.front().second; window_cpy-->0  && i>=0; i--) {
      int load_val = i >=0 ? i : sections->size()-i;
      required_sections.push_front(load_val);

    }
    return true;
  }

  bool KNNPolicy::operator()(std::deque< spatial_data>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window) {
    /*
     *  Initialise distances to each section
     */
    std::vector< std::pair< double, unsigned int > > distances;

    /*
     *  Calculate distances to all sections
     */
    for(unsigned int i=0; i<sections->size(); i++) {
      distances.push_back(std::make_pair(norm(std::make_pair(x, y), std::make_pair((*sections)[i].x, (*sections)[i].y) ), i));
    }

    /*
     *  Sort the distances
     */
    std::sort(distances.begin(), distances.end());
    std::vector< std::pair< double, unsigned int > >::iterator distances_iterator = distances.begin();

    /*
     *  Build up a list of required sections
     */
    required_sections.clear();
    for(int i=0; i<window && i<(int)distances.size(); i++) {
      required_sections.push_front(distances_iterator++->second);
    }

    return (!required_sections.empty());
  }
} // L3

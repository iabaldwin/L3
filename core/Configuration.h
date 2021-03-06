#pragma once

#include <vector>
#include <map>
#include <sstream>
#include <iterator>
#include <iostream>

#include <boost/filesystem.hpp>
#include <libconfig.h++>

#include "Datatypes.h"
#include "Dataset.h"
#include "Utils.h"

namespace L3
{
namespace Configuration
{

boost::filesystem::path configurationDirectory(void);

/*
 *Configuration structures
 */
struct LIDARParameters {
    LIDARParameters()
    {
      transform.resize(6);
    }

    std::string             name;
    std::vector<double>     transform;
    double                  frequency;
    bool                    inverted;
};

struct INSParameters
{
};

struct Configuration {

    Configuration() {
    }

    bool load(const std::string& target) {
      FILE* f = fopen(target.c_str(), "r" );
      if (!f)
        return false;
      config.read(f);
      return true;
    }

    boost::filesystem::path p;

    libconfig::Config config;
};

struct Mission : Configuration {

  Mission(const L3::Dataset& dataset) {
    // Load configuration directory
    p = L3::Configuration::configurationDirectory();
    // Add target
    p /= "missions";
    p /= dataset.name() + ".config";
    target = p.string();
    loadAll(target);
  }

  std::string locale;
  std::string target;
  std::string description;
  std::map< std::string, LIDARParameters > lidars;

  // Estimation
  std::string horizontal;
  std::string declined;

  Mission(const std::string& target) {
    this->target = target;
    loadAll(target);
  }

  bool loadAll(std::string target)
  {
    if (!load(target)) {
      throw std::runtime_error("Failed to load: [" + target + "]");
    }
    if (!loadDescription()) {
      throw std::runtime_error("Failed to load description: [" + target + "]");
    }
    if (!loadLIDARS()) {
      throw std::runtime_error("Failed to load LIDARS : [" + target + "]");
    }
    if (!loadEstimation()) {
      throw std::runtime_error("Failed to load estimation : [" + target + "]");
    }
    if (!loadLocale()) {
      throw std::runtime_error("Failed to load locale: [" + target + "]");
    }
    return true;
  }


  bool loadDescription();
  bool loadLocale();
  bool loadLIDARS();
  bool loadEstimation();

};

struct Locale : Configuration {

  Locale(const std::string& locale) : name(locale) {

    // Load configuration directory
    p = L3::Configuration::configurationDirectory();

    // Add target
    p /= "datums";
    p /= locale + ".config";

    if (!load(p.string()) && loadDatum())
      throw std::runtime_error("Failed to load: " + locale);
  }

  std::string name;
  double x_lower, x_upper, y_lower, y_upper, z;

  bool loadDatum()
  {
    std::stringstream ss;

    // X : Lower
    ss <<  "datum.X.lower";

    if (!config.lookupValue(ss.str(), x_lower))
      return false;

    ss.str(std::string());

    // X : Upper
    ss <<  "datum.X.upper";

    if (!config.lookupValue(ss.str(), x_upper))
      return false;

    ss.str(std::string());

    // Y : Lower
    ss <<  "datum.Y.lower";

    if (!config.lookupValue(ss.str(), y_lower))
      return false;

    ss.str(std::string());

    // Y : Upper
    ss <<  "datum.Y.upper";

    if (!config.lookupValue(ss.str(), y_upper))
      return false;

    return true;
  }

};

struct Begbroke : Locale {
    Begbroke() : Locale("begbroke_datum") {};
};

struct Woodstock : Locale {
    Woodstock() : Locale("woodstock_datum") {};
};

struct LocaleFactory {

  LocaleFactory() {
    locales.insert(std::make_pair("Woodstock", new Woodstock()));
    locales.insert(std::make_pair("Begbroke", new Begbroke()));
  }

  Locale* getLocale(std::string locale) {
    std::map< std::string, Locale* >::iterator it = locales.find( locale);

    if(it != locales.end()) {
      return it->second;
    }
    else {
      return nullptr;
    }
  }

  std::map< std::string, Locale* > locales;
};

/*
 *Conversion
 */
bool convert(const LIDARParameters& params, L3::SE3& calibration);

/*
 *  I/O
 */
std::ostream& operator<<(std::ostream& o, const Mission& mission);
std::ostream& operator<<(std::ostream& o, const std::pair< std::string, LIDARParameters> & parameters);

} // Configuration
} // L3

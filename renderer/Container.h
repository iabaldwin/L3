#pragma once

/*
 *  Containers
 */
namespace L3
{
  struct Container
  {
    boost::shared_ptr< L3::Dataset >                dataset;
    boost::shared_ptr< L3::Experience >             experience;
    boost::shared_ptr< L3::DatasetRunner >          runner;
    boost::shared_ptr< L3::Configuration::Mission > mission;
  };
} // L3

/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#pragma once

#include <string>
#include <boost/program_options.hpp>

class ProgramOptions
{
  public:

    ProgramOptions(const std::string& app_name);

    template<typename T>
    void addOptionalArgument(const std::string& name, const std::string& description, T& value);

    template<typename T>
    void addPositionalArgument(const std::string& name, const std::string& description, T& value);

    void addOptionalArgumentFlag(const std::string& name, const std::string& description);

    /**
     * @brief parse command line values.
     */
    void parse(int argc, char **argv);

    /**
     * @brief throws argument parsing exceptions.
     */
    void notify();

    /**
     * @brief count given values for an argument.
     */
    int count(const std::string& name) const;

    /**
     * @brief:
     *   Print usage message to stream.
     */
    friend std::ostream& operator << ( std::ostream& os, const ProgramOptions& program_options );

  private:

    std::string app_name_;

    size_t n_optional_arguments_;
    boost::program_options::options_description optional_arguments_;

    size_t n_positional_arguments_;
    boost::program_options::options_description hidden_arguments_;
    boost::program_options::positional_options_description positional_arguments_;

    boost::program_options::variables_map vm_;
};

// positional arguments son los argumentos del programa que no tienen una bandera (--bandera argumento) que los anuncia.

template<typename T>
void ProgramOptions::addPositionalArgument(const std::string& name, const std::string& description, T& value)
{
  hidden_arguments_.add_options()
    (name.c_str(), boost::program_options::value<T>(&value)->required(), description.c_str());

  // TODO: number of expected values is hardcoded to 1
  positional_arguments_.add(name.c_str(), 1);

  n_positional_arguments_++;
}

// optional_arguments son aquellos que necesitan una bandera. Ej: --grnd-poses path_to_ground_truth

template<typename T>
void ProgramOptions::addOptionalArgument(const std::string& name, const std::string& description, T& value )
{
  optional_arguments_.add_options()
    (name.c_str(), boost::program_options::value<T>(&value), description.c_str());

  n_optional_arguments_++;
}

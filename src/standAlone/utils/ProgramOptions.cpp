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

#include "ProgramOptions.hpp"

ProgramOptions::ProgramOptions(const std::string& app_name)
  : app_name_( app_name ), n_optional_arguments_(0), n_positional_arguments_(0)
{}


// addOptionalArgument con aridad para optional_arguments no se necesitan un argumento. Ej: --help
void ProgramOptions::addOptionalArgumentFlag(const std::string& name, const std::string& description)
{
  optional_arguments_.add_options()
    (name.c_str(), description.c_str());

  n_optional_arguments_++;
}

void ProgramOptions::parse(int argc, char **argv)
{
  boost::program_options::options_description all_options;
  all_options.add( optional_arguments_ );
  all_options.add( hidden_arguments_ );

  // throws on error
  boost::program_options::store(
    boost::program_options::command_line_parser(argc, argv)
    .options( all_options )
    .positional( positional_arguments_ )
    .run()
  , vm_);
}

void ProgramOptions::notify()
{
  boost::program_options::notify( vm_ );
}

int ProgramOptions::count(const std::string& name) const
{
  return vm_.count( name );
}

std::ostream& operator << ( std::ostream& os, const ProgramOptions& program_options )
{
  os << "Usage: " << program_options.app_name_ << " [OPTION]... ";

  for (size_t i=0; i<program_options.n_positional_arguments_; i++)
    os << "[" << program_options.positional_arguments_.name_for_position( i ) << "] ";


  os << "configuration file is a YML file" << std::endl;
  os << "image source must be passed in this way: path_to_image_directory" << std::endl;

  os << std::endl << std::endl;

  if ( program_options.optional_arguments_.options().size() + program_options.hidden_arguments_.options().size() > 0 )
  {
    os << "options" << std::endl << std::endl;

    os << /*program_options.hidden_arguments_ << */program_options.optional_arguments_;
  }

  return os;
}

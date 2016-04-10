/*******************************************************************************
 *
 *   Copyright (C) 2016 by Tom Koeppchen
 *   gtkoeppchen (at) googlemail (dot) com
 *   This file is part of stereocamcalib.
 *
 *   Stereocamcalib is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   Stereocamcalib is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with stereocamcalib.  If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <sstream>
#include <dirent.h>
#include <string>
#include <sys/stat.h>
#include <vector>

namespace scc {
  class Path {

  public:
      /** Constructors **/
      Path();
      explicit Path(std::string p);
      /** Destructor **/
      ~Path();

      /** Public functions **/
      std::string getPath();

      std::string getDirName();
      std::string getFile();
      std::string getFileName();
      std::string getFileExtension();

      Path getParentDir();
      Path getGrandParentDir();

      Path createDir(std::string dir);

  private:
      /** Attributes **/
      std::string path;

      /** Private functions **/
      bool hasTrailingSlash();
      std::string removeTrailingSlashFromPath();
      void appendTrailingSlash();
      void setPath(std::string path);
  };
}

#endif // PATH_H

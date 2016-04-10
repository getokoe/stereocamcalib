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

#include "path.h"

namespace scc {

  /**
  * Default constructor. Creates an path object with empty path attribute.
  */
  Path::Path() {
      path = "";
  }

  /**
  * Constructor.
  * @param p The attribute path is set for this value. Trailing slash means given
  *          path ends with dir, no trailing slash means given path ends with
  *          file.
  */
  Path::Path(std::string p) {
      path = p;
  }

  /**
  * Destructor.
  */
  Path::~Path() {

  }

  /**
  * Returns the attribute path.
  * @return
  */
  std::string Path::getPath() {
      return path;
  }

  /**
  * Returns the name of the directory the attribute path is pointing to.
  * @return The name of the directory as string or an empty string if the
  *          attribute path is pointing to a file.
  */
  std::string Path::getDirName() {
      std::string dir = "";
      if (hasTrailingSlash()) {
          dir = removeTrailingSlashFromPath();
          size_t found = dir.find_last_of("/\\");
          if (found != std::string::npos) {
              dir = dir.substr(found + 1, dir.size());
          }
      }
      return dir;
  }

  /**
  * Returns the name of the file the attribute path is pointing to.
  * @return The name of the file as string or an empty string if the attribute
  *          path is pointing to a directory.
  */
  std::string Path::getFile() {
      std::string file = "";
      if (!hasTrailingSlash()) {
          size_t found = path.find_last_of("/\\");
          if (found != std::string::npos) {
              file = path.substr(found + 1, path.size());
          }
      }
      return file;
  }

  /**
  * Returns the name of the file without its extension the attribute path is
  * pointing to.
  * @return The name of the file without its extension as string or an empty
  *          string if the attribute path is pointing to a directory.
  */
  std::string Path::getFileName() {
      std::string fileName = getFile();
      size_t found = fileName.find_last_of(".");
      if (found != std::string::npos) {
          fileName = fileName.substr(0, found);
      }
      return fileName;
  }

  /**
  * Returns the extension of the file the attribute path is pointing to.
  * @return The extension of the file as string or an empty string if the
  *          attribute path is pointing to a directory.
  */
  std::string Path::getFileExtension() {
      std::string fileExt = getFile();
      size_t found = fileExt.find_last_of(".");
      if (found != std::string::npos) {
          fileExt = fileExt.substr(found + 1, fileExt.size() - 1);
      }
      return fileExt;
  }

  /**
  * Returns a path object pointing to the parent directory of the actual path
  * object.
  * @return The new path object.
  */
  Path Path::getParentDir() {
      Path *p = new Path();
      std::string str = "";
      if (hasTrailingSlash()) {
          str = removeTrailingSlashFromPath();
      } else {
          str = getPath();
      }
      size_t found = str.find_last_of("/\\");
      if (found != std::string::npos) {
          p->setPath(str.substr(0, found + 1));
      }
      return *p;
  }

  /**
  * Returns a path object pointing to the grand parent directory of the actual
  * path object.
  * @return The new path object.
  */
  Path Path::getGrandParentDir() {
      Path parent = getParentDir();
      Path grand = parent.getParentDir();
      return grand;
  }

  /**
  * Creates a new directory and returns a path object pointing to this directory.
  * If the actual path object points to a file the new directory is created
  * inside the parent directory. Otherwise the new directory is created inside
  * the directory the actual path object points to.
  * @param dir The name of the directory to create.
  * @return The new path object.
  */
  Path Path::createDir(std::string dir) {
      Path *p = new Path();
      std::string d = "";
      if (!hasTrailingSlash()) {
          size_t found = path.find_last_of("/\\");
          if (found != std::string::npos) {
              d = path.substr(0, found + 1);
          }
      } else {
          d = path;
      }
      d.append(dir);
      if (opendir(d.c_str()) != NULL) {
          p->setPath(d);
      } else if (mkdir(d.c_str(),
                      S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) == 0) {
          p->setPath(d);
      }
      if (!p->hasTrailingSlash()) {
          p->appendTrailingSlash();
      }
      return *p;
  }

  /**
  * Checks whether the path attribute has a trainling slash or not.
  * @return True if a trailing slash exists, otherwise false.
  */
  bool Path::hasTrailingSlash() {
      bool retVal = true;
      size_t found = path.find_last_of("/\\");
      if (found != path.size() - 1 || found == std::string::npos) {
          retVal = false;
      }
      return retVal;
  }

  /**
  * Removes trailing slashes from the attribute path as long as there is one.
  * @return A string identical to atrribute path except for trailing slashes.
  */
  std::string Path::removeTrailingSlashFromPath() {
      std::string p = path;
      size_t found = p.find_last_of("/\\");
      while (found == p.size() - 1 && p.size() > 0) {
          p.erase(p.size() - 1);
          found = p.find_last_of("/\\");
      }
      return p;
  }

  /**
  * Appends a trailing slash to the attribute path.
  */
  void Path::appendTrailingSlash() {
      // TODO: make OS save
      path.append("/");
  }

  /**
  * Sets the attribute path to the given value.
  * @param p The string to set for the atrribute path.
  */
  void Path::setPath(std::string p) {
      path = p;
  }
}
/* Author: Masaki Murooka */

#pragma once

#include <sstream>
#include <string>

namespace MocapRosUtils
{
/** \brief Split a string to append a string list.
    \tparam Out type of iterator to append string list
    \param s input string
    \param delim delimiter
    \param result iterator to append string list
    \param skip_empty whether to skip empty element

    See https://stackoverflow.com/a/236803
*/
template<typename Out>
inline void splitStr(const std::string & s, char delim, Out result, bool skip_empty = false)
{
  std::istringstream iss(s);
  std::string item;
  while(std::getline(iss, item, delim))
  {
    if(skip_empty && item.empty())
    {
      continue;
    }

    *result++ = item;
  }
}

/** \brief Split a string to get a string list.
    \param s input string
    \param delim delimiter
    \param skip_empty whether to skip empty element

    See https://stackoverflow.com/a/236803
*/
inline std::vector<std::string> splitStr(const std::string & s, char delim, bool skip_empty = false)
{
  std::vector<std::string> elems;
  splitStr(s, delim, std::back_inserter(elems), skip_empty);
  return elems;
}
} // namespace MocapRosUtils

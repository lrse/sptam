#include <list>

template<typename T>
std::set<T> setUnion(const std::set<T>& s1, const std::set<T>& s2)
{
  std::set<T> ret;

  auto it1 = s1.begin();
  auto it2 = s2.begin();

  while (it1 != s1.end() or it2 != s2.end())
  {
    if ( it2==s2.end() or (it1!=s1.end() and *it1 < *it2) ) {
      ret.insert(ret.end(), *it1);
      it1++;
    } else {
      ret.insert(ret.end(), *it2);
      it2++;
    }
  }

  return ret;
}

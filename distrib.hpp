//
//  distrib.hpp
//  test1
//
//  Created by Bac Alexandra on 21/03/2022.
//

#ifndef distrib_hpp
#define distrib_hpp

#include <iostream>
using namespace std ;

template <typename T>
class distrib {
	vector<T> _data ;
	T _mean_sum, _min, _max, _stdev ;
public:
	distrib() : _mean_sum(0.), _min(0.), _max(0.), _stdev(0.)  { } ;
	distrib(vector<T> &vec) : _data(vec), _mean_sum(0.), _min(0.), _max(0.), _stdev(0.)
	{
		if (_data.size()>0)
		{
			_mean_sum = _data.at(0) ;
			_min = _data.at(0) ;
			_max = _data.at(0) ;
			for (int i=1; i<_data.size(); ++i)
			{
				_mean_sum += _data.at(i) ;
				if (_data.at(i) < _min)
					_min = _data.at(i) ;
				if (_data.at(i) > _max)
					_max = _data.at(i) ;
			}
		}
		update_stdev() ;
	} ;
	
	void update_stdev ()
	{
		T mean = _mean_sum / _data.size(), tmp ;
		for (int i=0; i<_data.size(); ++i)
		{
			tmp = _data.at(i) - mean ;
			_stdev += tmp*tmp ;
		}
		_stdev = sqrt(_stdev / _data.size()) ;
	} ;
	// Accesseurs
	T get_mean () { return _mean_sum/_data.size() ; } ;
	T get_stdev () { update_stdev() ; return _stdev ; } ;
	T get_min () { return _min ; } ;
	T get_max () { return _max ; } ;
	void add_data (T dat)
	{
		if (_data.size() == 0)
		{
			_min = dat ;
			_max = dat ;
		}
		else
		{
			if (dat < _min)
				_min = dat ;
			if (dat > _max)
				_max = dat ;
		}
		_data.push_back(dat) ; _mean_sum += dat ;
	}
	void clear ()
	{ _data.clear() ; _min = 0. ; _max = 0. ; _mean_sum = 0. ; _stdev = 0. ;}
};

#endif /* distrib_hpp */

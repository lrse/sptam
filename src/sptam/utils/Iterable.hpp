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

#ifndef ITERABLE_HPP_
#define ITERABLE_HPP_

#include <typeinfo>	// typeid
#include <memory>		// unique_ptr
#include <assert.h>	// assert

#include <set>
#include <map>
#include <list>
#include <vector>

/**
 * Abstract interface. Collection-specific implementations
 * should inherit from this.
 */
template<typename E>
class IteratorBase
{
	public:

		// virtual destructor
		virtual ~IteratorBase() {}

		virtual void operator ++ () = 0;

		virtual void operator ++ (int i) = 0;

		virtual E& operator * () = 0;

		virtual const E& operator * () const = 0;

		virtual IteratorBase* clone() const = 0;

		/**
		 * Check that the derived objects have compatible types, 
		 * then call the virtual comparison function 'equal'.
		 */
		bool operator == (const IteratorBase& other) const
		{
			return typeid( *this ) == typeid( other ) and equal( other );
		}

	protected:

		virtual bool equal(const IteratorBase& other) const = 0;
};

/**
 * Abstract interface. Collection-specific implementations
 * should inherit from this.
 */
template<typename E>
class ConstIteratorBase
{
	public:

		// virtual destructor
		virtual ~ConstIteratorBase() {}

		virtual void operator ++ () = 0;

		virtual void operator ++ (int i) = 0;

		virtual const E& operator * () const = 0;

		virtual ConstIteratorBase* clone() const = 0;

		/**
		 * Check that the derived objects have compatible types, 
		 * then call the virtual comparison function 'equal'.
		 */
		bool operator == (const ConstIteratorBase& other) const
		{
			return typeid( *this ) == typeid( other ) and equal( other );
		}

	protected:

		virtual bool equal(const ConstIteratorBase& other) const = 0;
};

/**
 * This class will be used as the abstract iterator interface
 * Internally it stores a pointer to a interface-compliant
 * and implementation-dependent object.
 */
template<typename E>
class Iterator {

	public:

		Iterator( std::unique_ptr< IteratorBase<E> > it_ptr )
		{
			it_iml_ptr_ = std::move( it_ptr );
		}

		Iterator(const Iterator<E>& other)
			: it_iml_ptr_( other.it_iml_ptr_->clone() )
		{}

		Iterator& operator = (const Iterator& other)
		{
			if (it_iml_ptr_ != other.it_iml_ptr_)
			{
				it_iml_ptr_ = other.it_iml_ptr_->clone();
			}
			return *this;
		}

		Iterator& operator ++ ()
		{
			++(*it_iml_ptr_);
			return *this;
		}

		Iterator operator ++ (int i) 
		{
			Iterator ret = *this;
			++(*this);
			return ret;
		}

		E& operator * ()
		{
			return *(*it_iml_ptr_);
		}

		const E& operator * () const
		{
			return *(*it_iml_ptr_);
		}

		bool operator == (const Iterator& other) const
		{
			return (it_iml_ptr_ == other.it_iml_ptr_) || (*it_iml_ptr_ == *other.it_iml_ptr_);
		}

		bool  operator != (const Iterator& other) const
		{
			return !(*this == other);
		}

	protected:

		std::unique_ptr< IteratorBase<E> > it_iml_ptr_;

		//~ friend ConstIterator<E>;
};

template<typename E>
class ConstIterator {

	public:

		ConstIterator( std::unique_ptr< ConstIteratorBase<E> > it_ptr )
		{
			it_iml_ptr_ = std::move( it_ptr );
		}

		//~ ConstIterator(const Iterator<E>& other)
			//~ : it_iml_ptr_( other.it_iml_ptr_->clone() )
		//~ {}

		ConstIterator(const ConstIterator& other)
			: it_iml_ptr_( other.it_iml_ptr_->clone() )
		{}

		ConstIterator& operator = (const ConstIterator& other)
		{
			if (it_iml_ptr_ != other.it_iml_ptr_)
			{
				it_iml_ptr_ = other.it_iml_ptr_->clone();
			}
			return *this;
		}

		ConstIterator& operator ++ ()
		{
			++(*it_iml_ptr_);
			return *this;
		}

		ConstIterator operator ++ (int i) 
		{
			ConstIterator ret = *this;
			++(*this);
			return ret;
		}

		const E& operator * () const
		{
			return *(*it_iml_ptr_);
		}

		bool operator == (const ConstIterator& other) const
		{
			return (it_iml_ptr_ == other.it_iml_ptr_) || (*it_iml_ptr_ == *other.it_iml_ptr_);
		}

		bool operator != (const ConstIterator& other) const
		{
			return !(*this == other);
		}

	protected:

		std::unique_ptr< ConstIteratorBase<E> > it_iml_ptr_;
};

////////////////////////////////////////////////////////////////////////
// Interface for iterable objects
////////////////////////////////////////////////////////////////////////

/**
 * Interface for an iterable object
 */
template<typename E>
class IterableBase
{
  public:

    virtual Iterator<E> begin() = 0;

		virtual ConstIterator<E> begin() const = 0;

    virtual Iterator<E> end() = 0;

    virtual ConstIterator<E> end() const = 0;

		virtual bool empty() const = 0;

		virtual size_t size() const = 0;
};

template<typename E>
class ConstIterableBase
{
  public:

		virtual ConstIterator<E> begin() const = 0;

    virtual ConstIterator<E> end() const = 0;

    virtual bool empty() const = 0;

		virtual size_t size() const = 0;
};

template<typename E>
class Iterable
{
  public:

    Iterable( std::unique_ptr< IterableBase<E> > collection_ptr )
		{
			collection_impl_ptr_ = std::move( collection_ptr );
		}

    Iterator<E> begin()
    {
			return collection_impl_ptr_->begin();
		}

		ConstIterator<E> begin() const
		{
			return collection_impl_ptr_->begin();
		}

    Iterator<E> end()
    {
			return collection_impl_ptr_->end();
		}

		ConstIterator<E> end() const
		{
			return collection_impl_ptr_->end();
		}

		virtual bool empty() const
		{
			return collection_impl_ptr_->empty();
		}

		virtual size_t size() const
		{
			return collection_impl_ptr_->size();
		}

  private:

    std::unique_ptr< IterableBase<E> > collection_impl_ptr_;
};

template<typename E>
class ConstIterable
{
  public:

    ConstIterable( std::unique_ptr< ConstIterableBase<E> > collection_ptr )
		{
			collection_impl_ptr_ = std::move( collection_ptr );
		}

		ConstIterator<E> begin() const
		{
			return collection_impl_ptr_->begin();
		}

		ConstIterator<E> end() const
		{
			return collection_impl_ptr_->end();
		}

		virtual bool empty() const
		{
			return collection_impl_ptr_->empty();
		}

		virtual size_t size() const
		{
			return collection_impl_ptr_->size();
		}

  private:

    std::unique_ptr< ConstIterableBase<E> > collection_impl_ptr_;
};

////////////////////////////////////////////////////////////////////////
// preprocessor macro for generic containers
////////////////////////////////////////////////////////////////////////

#define CONCAT(x, y) x ## y

#define DEFINE_ITERATOR( PREFIX, CONTAINER ) \
template<typename E> \
class PREFIX ## Iterator : public IteratorBase<E> \
{ \
	public: \
\
		PREFIX ## Iterator(typename CONTAINER<E>::iterator it) \
			: it_( it ) \
		{} \
\
		virtual void operator ++ () \
		{ it_++; } \
\
		virtual void operator ++ (int i) \
		{ assert( false ); } \
\
		virtual E& operator * () \
		{ return *it_; } \
\
		const virtual E& operator * () const \
		{ return *it_; } \
\
		virtual IteratorBase<E>* clone() const \
		{ return new PREFIX ## Iterator( it_ ); } \
\
	protected: \
\
		virtual bool equal(const IteratorBase<E>& other) const \
		{ return it_ == ((PREFIX ## Iterator&)other).it_; } \
\
	private: \
\
		typename CONTAINER<E>::iterator it_; \
};

#define DEFINE_CONST_ITERATOR( PREFIX, CONTAINER ) \
template<typename E> \
class Const ## PREFIX ## Iterator : public ConstIteratorBase<E> \
{ \
	public: \
 \
		Const ## PREFIX ## Iterator(typename CONTAINER<E>::const_iterator it) \
			: it_( it ) \
		{} \
 \
		virtual void operator ++ () \
		{ it_++; } \
 \
		virtual void operator ++ (int i) \
		{ assert( false ); } \
 \
		const virtual E& operator * () const \
		{ return *it_; } \
 \
	protected: \
 \
		virtual bool equal(const ConstIteratorBase<E>& other) const \
		{ \
			return it_ == ((Const ## PREFIX ## Iterator&)other).it_; \
		} \
 \
		virtual ConstIteratorBase<E>* clone() const \
		{ \
			return new Const ## PREFIX ## Iterator( it_ ); \
		} \
 \
	private: \
 \
		typename CONTAINER<E>::const_iterator it_; \
};

#define DEFINE_ITERABLE( PREFIX, CONTAINER ) \
template<typename E> \
class PREFIX ## Iterable : public IterableBase<E> \
{ \
  public: \
 \
    virtual Iterator<E> begin() \
    { \
			return Iterator<E>( std::unique_ptr< PREFIX ## Iterator<E> >( new PREFIX ## Iterator<E>( container_.begin() ) ) ); \
		} \
 \
		virtual ConstIterator<E> begin() const \
		{ \
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new Const ## PREFIX ## Iterator<E>( container_.begin() ) )); \
		} \
 \
    virtual Iterator<E> end() \
    { \
			return Iterator<E>( std::unique_ptr< PREFIX ## Iterator<E> >( new PREFIX ## Iterator<E>( container_.end() ) ) ); \
		} \
 \
		virtual ConstIterator<E> end() const \
		{ \
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new Const ## PREFIX ## Iterator<E>( container_.end() ) )); \
		} \
 \
		virtual bool empty() const \
		{ return container_.empty(); } \
 \
		virtual size_t size() const \
		{ return container_.size(); } \
 \
    static Iterable<E> from( CONTAINER<E>& container ) \
    { \
			return Iterable<E>( std::unique_ptr< IterableBase<E> >( new PREFIX ## Iterable<E>( container ) ) ); \
		} \
 \
  protected: \
 \
    PREFIX ## Iterable( CONTAINER<E>& container ) : container_( container ) \
		{} \
 \
    CONTAINER<E>& container_; \
};

#define DEFINE_CONST_ITERABLE( PREFIX, CONTAINER ) \
template<typename E> \
class Const ## PREFIX ## Iterable : public ConstIterableBase<E> \
{ \
  public: \
 \
		virtual ConstIterator<E> begin() const \
		{ \
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new Const ## PREFIX ## Iterator<E>( container_.begin() ) )); \
		} \
 \
		virtual ConstIterator<E> end() const \
		{ \
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new Const ## PREFIX ## Iterator<E>( container_.end() ) )); \
		} \
 \
		virtual bool empty() const \
		{ return container_.empty(); } \
 \
		virtual size_t size() const \
		{ return container_.size(); } \
 \
    static ConstIterable<E> from( const CONTAINER<E>& container ) \
    { \
			return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new Const ## PREFIX ## Iterable<E>( container ) ) ); \
		} \
 \
  protected: \
 \
    Const ## PREFIX ## Iterable( const CONTAINER<E>& container ) : container_( container ) \
		{} \
 \
    const CONTAINER<E>& container_; \
};

////////////////////////////////////////////////////////////////////////
// std::list<E> iterator implementation
////////////////////////////////////////////////////////////////////////

DEFINE_ITERATOR( List, std::list )
DEFINE_CONST_ITERATOR( List, std::list )
DEFINE_ITERABLE( List, std::list )
DEFINE_CONST_ITERABLE( List, std::list )

////////////////////////////////////////////////////////////////////////
// std::vector<E> iterator implementation
////////////////////////////////////////////////////////////////////////

DEFINE_ITERATOR( Vector, std::vector )
DEFINE_CONST_ITERATOR( Vector, std::vector )
DEFINE_ITERABLE( Vector, std::vector )
DEFINE_CONST_ITERABLE( Vector, std::vector )

////////////////////////////////////////////////////////////////////////
// std::set<E> iterator implementation
////////////////////////////////////////////////////////////////////////

/*template<typename E, typename CMP>
class SetIterator : public IteratorBase<E>
{
	public:

		SetIterator(typename std::set<E, CMP>::iterator it)
			: it_( it )
		{}

		virtual void operator ++ ()
		{ it_++; }

		virtual void operator ++ (int i)
		{ assert( false ); }

		virtual E& operator * ()
		{ return *it_; }

		const virtual E& operator * () const
		{ return *it_; }

		virtual IteratorBase<E>* clone() const
		{ return new SetIterator( it_ ); }

	protected:

		virtual bool equal(const IteratorBase<E>& other) const
		{ return it_ == ((SetIterator&)other).it_; }

	private:

		typename std::set<E, CMP>::iterator it_;
};*/

template<typename E, typename CMP = std::less<E> >
class ConstSetIterator : public ConstIteratorBase<E>
{
	public:

		ConstSetIterator(typename std::set<E, CMP>::const_iterator it)
			: it_( it )
		{}

		virtual void operator ++ ()
		{ it_++; }

		virtual void operator ++ (int i)
		{ assert( false ); }

		const virtual E& operator * () const
		{ return *it_; }

	protected:

		virtual bool equal(const ConstIteratorBase<E>& other) const
		{
			return it_ == ((ConstSetIterator&)other).it_;
		}

		virtual ConstIteratorBase<E>* clone() const
		{
			return new ConstSetIterator( it_ );
		}

	private:

		typename std::set<E, CMP>::const_iterator it_;
};

/*template<typename E, typename CMP>
class SetIterable : public IterableBase<E>
{
  public:

    virtual Iterator<E> begin()
    {
			return Iterator<E>( std::unique_ptr< SetIterator<E, CMP> >( new SetIterator<E, CMP>( container_.begin() ) ) );
		}

		virtual ConstIterator<E> begin() const
		{
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSetIterator<E, CMP>( container_.begin() ) ));
		}

    virtual Iterator<E> end()
    {
			return Iterator<E>( std::unique_ptr< SetIterator<E, CMP> >( new SetIterator<E, CMP>( container_.end() ) ) );
		}

		virtual ConstIterator<E> end() const
		{
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSetIterator<E, CMP>( container_.end() ) ));
		}

		virtual bool empty() const
		{ return container_.empty(); }

		virtual size_t size() const
		{ return container_.size(); }

    static Iterable<E> from( std::set<E, CMP>& container )
    {
			return Iterable<E>( std::unique_ptr< IterableBase<E> >( new SetIterable<E, CMP>( container ) ) );
		}

  protected:

    SetIterable( std::set<E, CMP>& container ) : container_( container )
		{}

    std::set<E, CMP>& container_;
};*/

template<typename E, typename CMP = std::less<E> >
class ConstSetIterable : public ConstIterableBase<E>
{
  public:

		virtual ConstIterator<E> begin() const
		{
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSetIterator<E, CMP>( container_.begin() ) ));
		}

		virtual ConstIterator<E> end() const
		{
			return ConstIterator<E>(std::unique_ptr< ConstIteratorBase<E> >( new ConstSetIterator<E, CMP>( container_.end() ) ));
		}

		virtual bool empty() const
		{ return container_.empty(); }

		virtual size_t size() const
		{ return container_.size(); }

    static ConstIterable<E> from( const std::set<E, CMP>& container )
    {
			return ConstIterable<E>( std::unique_ptr< ConstIterableBase<E> >( new ConstSetIterable<E, CMP>( container ) ) );
		}

  protected:

    ConstSetIterable( const std::set<E, CMP>& container ) : container_( container )
		{}

    const std::set<E, CMP>& container_;
};

//~ DEFINE_ITERATOR( Set, std::set )
//~ DEFINE_CONST_ITERATOR( Set, std::set )
//~ DEFINE_ITERABLE( Set, std::set )
//~ DEFINE_CONST_ITERABLE( Set, std::set )

////////////////////////////////////////////////////////////////////////
// std::map<K,V> key-iterator implementation
////////////////////////////////////////////////////////////////////////

template<typename K, typename V>
class MapKeyIterator : public IteratorBase<K>
{
	public:

		MapKeyIterator(typename std::map<K, V>::iterator it)
			: it_( it )
		{}

		virtual void operator ++ ()
		{
			it_++;
		}

		virtual void operator ++ (int i)
		{
			it_++(i);
		}

		virtual K& operator * ()
		{
			return it_->first;
		}

		const virtual K& operator * () const
		{
			return it_->first;
		}

		virtual IteratorBase<K>* clone() const
		{
			return new MapKeyIterator( it_ );
		}

	protected:

		virtual bool equal(const IteratorBase<K>& other) const
		{
			return it_ == ((MapKeyIterator&)other).it_;
		}

	private:

		typename std::map<K, V>::iterator it_;
};

template<typename K, typename V>
class ConstMapKeyIterator : public ConstIteratorBase<K>
{
	public:

		ConstMapKeyIterator(typename std::map<K, V>::const_iterator it)
			: it_( it )
		{}

		virtual void operator ++ ()
		{
			it_++;
		}

		virtual void operator ++ (int i)
		{
			assert( false );
		}

		const virtual K& operator * () const
		{
			return it_->first;
		}

	protected:

		virtual bool equal(const ConstIteratorBase<K>& other) const
		{
			return it_ == ((ConstMapKeyIterator&)other).it_;
		}

		virtual ConstIteratorBase<K>* clone() const
		{
			return new ConstMapKeyIterator( it_ );
		}

	private:

		typename std::map<K, V>::const_iterator it_;
};

template<typename K, typename V>
class MapKeyIterable : public IterableBase<K>
{
  public:

    virtual Iterator<K> begin()
    {
			return Iterator<K>( MapKeyIterator<K, V>( map_.begin() ) );
		}

		virtual ConstIterator<K> begin() const
		{
			return ConstIterator<K>(std::unique_ptr< ConstIteratorBase<K> >( new ConstMapKeyIterator<K, V>( map_.begin() ) ));
		}

    virtual Iterator<K> end()
    {
			return Iterator<K>( MapKeyIterator<K, V>( map_.end() ) );
		}

		virtual ConstIterator<K> end() const
		{
			return ConstIterator<K>(std::unique_ptr< ConstIteratorBase<K> >( new ConstMapKeyIterator<K, V>( map_.end() ) ));
		}

		virtual bool empty() const
		{
			return map_.empty();
		}

		virtual size_t size() const
		{
			return map_.size();
		}

    static Iterable<K> from( const std::map<K, V>& map )
    {
			return Iterable<K>( std::unique_ptr< IterableBase<K> >( new MapKeyIterable<K, V>( map ) ) );
		}

  protected:

    MapKeyIterable( std::map<K, V>& map ) : map_( map )
		{}

    std::map<K, V>& map_;
};

template<typename K, typename V>
class ConstMapKeyIterable : public ConstIterableBase<K>
{
  public:

		virtual ConstIterator<K> begin() const
		{
			return ConstIterator<K>(std::unique_ptr< ConstIteratorBase<K> >( new ConstMapKeyIterator<K, V>( map_.begin() ) ));
		}

		virtual ConstIterator<K> end() const
		{
			return ConstIterator<K>(std::unique_ptr< ConstIteratorBase<K> >( new ConstMapKeyIterator<K, V>( map_.end() ) ));
		}

		virtual bool empty() const
		{
			return map_.empty();
		}

		virtual size_t size() const
		{
			return map_.size();
		}

    static ConstIterable<K> from( const std::map<K, V>& map )
    {
			return ConstIterable<K>( std::unique_ptr< ConstIterableBase<K> >( new ConstMapKeyIterable<K, V>( map ) ) );
		}

  protected:

    ConstMapKeyIterable( const std::map<K, V>& map ) : map_( map )
		{}

    const std::map<K, V>& map_;
};

////////////////////////////////////////////////////////////////////////
// std::map<K,V> value-iterator implementation
////////////////////////////////////////////////////////////////////////

template<typename K, typename V>
class MapValueIterator : public IteratorBase<V>
{
	public:

		MapValueIterator(typename std::map<K, V>::iterator it)
			: it_( it )
		{}

		virtual void operator ++ ()
		{
			it_++;
		}

		virtual void operator ++ (int i)
		{
			it_++(i);
		}

		virtual V& operator * ()
		{
			return it_->second;
		}

		const virtual V& operator * () const
		{
			return it_->second;
		}

		virtual IteratorBase<V>* clone() const
		{
			return new MapValueIterator( it_ );
		}

	protected:

		virtual bool equal(const IteratorBase<V>& other) const
		{
			return it_ == ((MapValueIterator&)other).it_;
		}

	private:

		typename std::map<K, V>::iterator it_;
};

template<typename K, typename V>
class ConstMapValueIterator : public ConstIteratorBase<V>
{
	public:

		ConstMapValueIterator(typename std::map<K, V>::const_iterator it)
			: it_( it )
		{}

		virtual void operator ++ ()
		{
			it_++;
		}

		virtual void operator ++ (int i)
		{
			assert( false );
		}

		const virtual V& operator * () const
		{
			return it_->second;
		}

	protected:

		virtual bool equal(const ConstIteratorBase<V>& other) const
		{
			return it_ == ((ConstMapValueIterator&)other).it_;
		}

		virtual ConstIteratorBase<V>* clone() const
		{
			return new ConstMapValueIterator( it_ );
		}

	private:

		typename std::map<K, V>::const_iterator it_;
};

template<typename K, typename V>
class MapValueIterable : public IterableBase<V>
{
  public:

    virtual Iterator<V> begin()
    {
			return Iterator<V>( MapValueIterator<K, V>( map_.begin() ) );
		}

		virtual ConstIterator<V> begin() const
		{
			return ConstIterator<V>(std::unique_ptr< ConstIteratorBase<V> >( new ConstMapValueIterator<K, V>( map_.begin() ) ));
		}

    virtual Iterator<V> end()
    {
			return Iterator<V>( MapValueIterator<K, V>( map_.end() ) );
		}

		virtual ConstIterator<V> end() const
		{
			return ConstIterator<V>(std::unique_ptr< ConstIteratorBase<V> >( new ConstMapValueIterator<K, V>( map_.end() ) ));
		}

		virtual bool empty() const
		{
			return map_.empty();
		}

		virtual size_t size() const
		{
			return map_.size();
		}

    static Iterable<V> from( const std::map<K, V>& map )
    {
			return Iterable<V>( std::unique_ptr< IterableBase<V> >( new MapValueIterable<K, V>( map ) ) );
		}

  protected:

    MapValueIterable( std::map<K, V>& map ) : map_( map )
		{}

    std::map<K, V>& map_;
};

template<typename K, typename V>
class ConstMapValueIterable : public ConstIterableBase<V>
{
  public:

		virtual ConstIterator<V> begin() const
		{
			return ConstIterator<V>(std::unique_ptr< ConstIteratorBase<V> >( new ConstMapValueIterator<K, V>( map_.begin() ) ));
		}

		virtual ConstIterator<V> end() const
		{
			return ConstIterator<V>(std::unique_ptr< ConstIteratorBase<V> >( new ConstMapValueIterator<K, V>( map_.end() ) ));
		}

		virtual bool empty() const
		{
			return map_.empty();
		}

		virtual size_t size() const
		{
			return map_.size();
		}

    static ConstIterable<V> from( const std::map<K, V>& map )
    {
			return ConstIterable<V>( std::unique_ptr< ConstIterableBase<V> >( new ConstMapValueIterable<K, V>( map ) ) );
		}

  protected:

    ConstMapValueIterable( const std::map<K, V>& map ) : map_( map )
		{}

    const std::map<K, V>& map_;
};

#endif // ITERABLE_HPP_

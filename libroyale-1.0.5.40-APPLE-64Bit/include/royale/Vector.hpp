/****************************************************************************\
* Copyright (C) 2015 Infineon Technologies
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
\****************************************************************************/

#pragma once

#include <royale/Definitions.hpp>
#include <royale/Iterator.hpp>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>
#include <map>
#include <royale/Pair.hpp>

namespace royale
{
    template<class T>
    class Vector
    {
        using V_TYPE = char;

    public:
        /**
        * Iterator definitions
        */
        typedef royale_iterator<std::random_access_iterator_tag, T> iterator;
        typedef royale_const_iterator<std::random_access_iterator_tag, T> const_iterator;
        typedef royale_reverse_iterator<std::random_access_iterator_tag, T> reverse_iterator;
        typedef royale_const_reverse_iterator<std::random_access_iterator_tag, T> const_reverse_iterator;

        /**
        * General constructor, which does not allocate memory and sets everything to it's default
        */
        DllExport Vector<T>();

        /**
        * Constructor which already performs an allocation of memory during execution
        */
        DllExport Vector<T> (size_t allocationSize);

        /**
        * Copy-Constructor for royale compliant vector
        * which allows creation of a royale compliant vector from
        * another royale compliant vector - (NOTE: performs a deep copy!)
        *
        * \param v The royale vector which's memory shall be copied
        */
        DllExport Vector<T> (const Vector<T> &v);

        /**
        * Move-Constructor for royale compliant vector
        * which allows creation of a royale compliant vector by moving memory
        * (NOTE: performs a shallow copy!)
        *
        * \param v The royale vector which's memory shall be moved
        */
        DllExport Vector<T> (Vector<T> &&v);

        /**
        * Copy-Constructor for STL compliant vector (std::vector)
        * It allows creation of a royale compliant vector from
        * a STL compliant vector - (NOTE: performs a deep copy!)
        *
        * \param v The STL vector to copy
        */
        DllExport Vector<T> (const std::vector<T> &v);

        /**
        * Initializer list initialization to initialize a vector
        *
        * \param list The list of values
        */
        DllExport Vector<T> (const std::initializer_list<T> &list);

        /**
        * Destructor
        *
        * Clears the vectors allocated memory by performing deletion
        */
        DllExport virtual ~Vector<T>();

        /**
        * Returns the actual size of the vector (this is the used amount of
        * slots in the allocated area)
        *
        * \return size_t The amount of the actual used memory slots within the vector
        */
        DllExport size_t size() const;

        /**
        * Checks if the vector is empty
        *
        * \return bool Returns true if the vector is empty - otherwise false
        */
        DllExport bool empty() const;

        /**
        * Returns the amount of allocated slots which are maintained by the vector
        * These allocated slots might be used or unused (refer to size() for checking the size()
        * itself)
        *
        * \return size_t The amount of allocated slots for the element type which is bound to the vector
        */
        DllExport size_t capacity() const;

        /**
        * Returns a direct pointer to the memory array used internally by the vector to store its owned elements.
        * Because elements in the vector are guaranteed to be stored in contiguous storage locations in the same order as
        * represented by the vector, the pointer retrieved can be offset to access any element in the array.
        *
        * \return T* A pointer to the first element in the array used internally by the vector.
        */
        DllExport T *data();

        /**
        * Returns a direct pointer to the memory array used internally by the vector to store its owned elements.
        * Because elements in the vector are guaranteed to be stored in contiguous storage locations in the same order as
        * represented by the vector, the pointer retrieved can be offset to access any element in the array.
        *
        * \return T* A pointer to the first element in the array used internally by the vector.
        */
        DllExport const T *data() const;

        /**
        * Returns a reference to the first element in the vector.
        * This member function returns a direct reference to the first element in the vector
        * Calling this function on an empty vector will result in a std::out_of_range exception.
        *
        * \return T& A reference to the last element in the vector.
        * \throws std::out_of_range Exception if the vector is empty
        */
        DllExport T &front();
        DllExport const T &front() const;

        /**
        * Returns an iterator to the first position
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator An iterator pointing to the begin of the vector.
        */
        DllExport iterator begin()
        {
            return iterator (reinterpret_cast<T *> (m_data.get()));
        }

        DllExport const_iterator begin() const
        {
            return const_iterator (reinterpret_cast<T *> (m_data.get()));
        }

        /**
        * Returns an iterator to the last position
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator An iterator pointing to the end of the vector.
        */
        DllExport iterator end()
        {
            return iterator (reinterpret_cast<T *> (m_data.get()) + m_actualSize);
        }

        DllExport const_iterator end() const
        {
            return const_iterator (reinterpret_cast<T *> (m_data.get()) + m_actualSize);
        }

        /**
        * Returns an iterator to the last position (reverse begin)
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator An iterator pointing to the reverse begin of a vector (!= end())
        */
        DllExport reverse_iterator rbegin()
        {
            return reverse_iterator (reinterpret_cast<T *> (m_data.get()) + m_actualSize - 1);
        }

        DllExport const_reverse_iterator rbegin() const
        {
            return const_reverse_iterator (reinterpret_cast<T *> (m_data.get()) + m_actualSize - 1);
        }

        /**
        * Returns an iterator to the last position (reverse end)
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator An iterator pointing to the reverse end of a vector (!= begin())
        */
        DllExport reverse_iterator rend()
        {
            return reverse_iterator (reinterpret_cast<T *> (m_data.get()) - 1);
        }

        DllExport const_reverse_iterator rend() const
        {
            return const_reverse_iterator (reinterpret_cast<T *> (m_data.get()) - 1);
        }

        /**
        * Returns an constant iterator to the first position
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator An iterator pointing to the begin of the vector.
        */
        DllExport const_iterator cbegin()
        {
            return const_iterator (begin());
        }

        DllExport const_iterator cbegin() const
        {
            return const_iterator (begin());
        }

        /**
        * Returns an constant iterator to the last position
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator An iterator pointing to the end of the vector.
        */
        DllExport const_iterator cend()
        {
            return const_iterator (end());
        }

        DllExport const_iterator cend() const
        {
            return const_iterator (end());
        }

        /**
        * Returns a constant reverse iterator to the first position
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator A constant iterator pointing to the reverse begin of the vector.
        */
        DllExport const_reverse_iterator crbegin()
        {
            return const_reverse_iterator (rbegin());
        }

        DllExport const_reverse_iterator crbegin() const
        {
            return const_reverse_iterator (rbegin());
        }

        /**
        * Returns a constant reverse iterator to the last position
        * Calling this function on an empty vector will result in undefined behavior
        *
        * \return iterator A constant iterator pointing to the reverse end of the vector.
        */
        DllExport const_reverse_iterator crend()
        {
            return const_reverse_iterator (rend());
        }

        DllExport const_reverse_iterator crend() const
        {
            return const_reverse_iterator (rend());
        }

        /**
        * Returns a reference to the last element in the vector.
        * This member function returns a direct reference to the last element in the vector
        * Calling this function on an empty vector will result in a std::out_of_range exception.
        *
        * \return T& A reference to the last element in the vector.
        * \throws std::out_of_range Exception if the vector is empty
        */
        DllExport T &back();
        DllExport const T &back() const;

        /**
        * Adds a new element at the end of the vector, after its current last element.
        * The content of v is copied to the new element (NOTE: a deep copy is performed!).
        * This effectively increases the container size by one, which causes an automatic reallocation
        * of the allocated storage space if -and only if- the new vector size surpasses the current vector capacity.
        */
        DllExport void push_back (const T &v);
        DllExport void push_back (T &&v);

        /**
        * Removes the last element in the vector, effectively reducing the container size by one.
        * The last element is destroyed by calling it's destructor; the size is reduced by one BUT
        * there is no reallocation performed to resize the vector to it's contents or to reduce the
        * the vectors capacity by one.
        * The allocated space remains the same.
        */
        DllExport void pop_back();

        /**
        * Returns a reference to the element at position index in the vector container.
        * A similar member function, at(), has the same behavior as this operator function,
        * except that at() is bound-checked and signals if the requested position is
        * out of range by throwing an out_of_range exception.
        *
        * \param index The index to access within the vectors storage
        * \return T& The Reference to the elements at the specified position in the vector
        */
        DllExport T &operator[] (size_t index);

        /**
        * Returns a reference to the element at position index in the vector container.
        * A similar member function, at(), has the same behavior as this operator function,
        * except that at() is bound-checked and signals if the requested position is
        * out of range by throwing an out_of_range exception.
        *
        * \param index The index to access within the vectors storage
        * \return T& The Reference to the elements at the specified position in the vector
        */
        DllExport const T &operator[] (size_t index) const;

        /**
        * The function automatically checks whether index is within the bounds
        * of valid elements in the vector, throwing an out_of_range exception
        * if it is not (i.e., if index is greater or equal than its size).
        * This is in contrast with member operator[], that does not check against bounds.
        *
        * \param index The index to access within the vectors storage
        * \return T& The Reference to the elements at the specified position in the vector
        */
        DllExport T &at (size_t index);

        /**
        * The function automatically checks whether index is within the bounds
        * of valid elements in the vector, throwing an out_of_range exception
        * if it is not (i.e., if index is greater or equal than its size).
        * This is in contrast with member operator[], that does not check against bounds.
        *
        * \param index The index to access within the vectors storage
        * \return T& The Reference to the elements at the specified position in the vector
        */
        DllExport const T &at (size_t index) const;

        /**
        * Assign another royale compliant vector.
        * Assigns new contents to the container, replacing its current contents,
        * and modifying its size accordingly.
        * This method copies all elements held by v into the container
        *
        * \param v A vector of the same storage type
        * \return Vector<T>& Returns *this
        */
        DllExport Vector<T> &operator= (Vector<T> v);

        /**
        * Assign the contents of an STL compliant vector container by
        * replacing container's current contents if necessary
        * and modifying its size accordingly.
        * This method copies all elements held by v into the container
        *
        * \param v An STL compliant vector of the same storage type
        * \return Vector<T>& Returns *this
        */
        DllExport Vector<T> &operator= (const std::vector<T> &v);

        /**
        * Checks equality with an STL compliant vector
        *
        * \param v An STL compliant vector of the same storage type
        * \return bool Returns true if they are equal and false is they are not
        */
        DllExport bool operator== (const std::vector<T> &v) const;

        /**
        * Checks equality with a royale compliant vector
        *
        * \param v An royale compliant vector of the same storage type
        * \return bool Returns true if they are equal and false is they are not
        */
        DllExport bool operator== (const Vector<T> &v) const;

        /**
        * Checks unequality with an STL compliant vector
        *
        * \param v An STL compliant vector of the same storage type
        * \return bool Returns false if they are equal and true is they are not
        */
        DllExport bool operator!= (const std::vector<T> &v) const;

        /**
        * Checks unequality with a royale compliant vector
        *
        * \param v An royale compliant vector of the same storage type
        * \return bool Returns false if they are equal and true is they are not
        */
        DllExport bool operator!= (const Vector<T> &v) const;

        /**
        * Removes all elements from the vector (which are destroyed), leaving the container with a size of 0.
        * A reallocation is not performed and the vector's capacity is destroyed (everything is freed).
        */
        DllExport void clear();

        /**
        * Modifies the vector to the given allocation size and initializes the elements (it may shrink)
        *
        * Creates any amount of elements (allocates the memory already)
        * and moves the existing elements to these slots; afterwards the old space is dumped.
        *
        * If the given newSize is smaller than the already used slots, the vector will shrink.
        * This means that all elements which are not covered within this capacity (the last ones) will be
        * deleted.
        *
        * \param newSize The amount of slots to remain in the vector (might shrink or enlarge the vector)
        */
        DllExport void resize (size_t newSize);

        /**
        * Modifies the vector to the given allocation size and initializes the elements (it may shrink)
        *
        * Creates any amount of elements (allocates the memory already)
        * and moves the existing elements to these slots; afterwards the old space is dumped.
        *
        * If the given newSize is smaller than the already used slots, the vector will shrink.
        * This means that all elements which are not covered within this capacity (the last ones) will be
        * deleted.
        *
        * \param newSize The amount of slots to remain in the vector (might shrink or enlarge the vector)
        * \param initVal The initializer value by which the vector shall be initialized
        */
        DllExport void resize (size_t newSize, T initVal);

        /**
        * Extends the vector to a higher allocation size and allocates the buffers
        *
        * Reserves any amount of free allocation slots (allocates the memory already) to be later
        * used for the element-types bound to the given royale vector.
        *
        * If the given size to reserve is smaller than the already reserved space, then the function
        * return immediately; otherwise the necessary memory allocation is performed and the size is
        * extended to "size"
        +
        * \param size The size (number of element-types) of elements that should be allocated.
        */
        DllExport void reserve (size_t size);

        /**
        * Shrinks the vector's allocation to it's size
        * Changes the size of the allocated buffer to the vector's size
        * this may result in freeing unneeded memory allocation.
        */
        DllExport void shrink_to_fit();

        /**
        * Converts the given std::vector (STL) to the vector type used by the royale API
        *
        * \param v The STL vector which should be converted to the royale API vector format
        * \return The royale API compliant vector format
        */
        static DllExport Vector<T> fromStdVector (const std::vector<T> &v);

        /**
        * User convenience function to allow conversion to std::vector
        * which might be used outside the library by the application for further processing
        *
        * \return std::vector containing the items of the
        */
        DllExport std::vector<T> toStdVector();
        DllExport const std::vector<T> toStdVector() const;

        /**
        * User convenience function to allow conversion to std::vector
        * which might be used if the royale compliant Vector is a const
        *
        * \return std::vector containing the items of the
        */
        static DllExport std::vector<T> toStdVector (const Vector<T> &v);

        /**
        * User convenience function to allow conversion from std::map
        * which might be used inside the library for further processing
        *
        * \param stdMap The STL compliant map which shall be converted to the API style vector
        * \return Vector<pair> Returned is a vector of pairs - each pair is holding one entry of the
        *                      converted std::map
        */
        template<typename X, typename Y>
        static DllExport Vector< Pair<X, Y> > fromStdMap (const std::map<X, Y> &stdMap)
        {
            Vector< Pair<X, Y> > tempVector (stdMap.size());
            for (auto &mapItem : stdMap)
            {
                tempVector.push_back (Pair<X, Y> (mapItem.first, mapItem.second));
            }
            return tempVector;
        }

        /**
        * User convenience function to allow conversion to std::map
        * which might be used outside the library by the application for further processing
        *
        * \return std::map containing the items of the Vector<pairs>
        */
        template<typename X, typename Y>
        DllExport std::map<X, Y> toStdMap()
        {
            std::map<X, Y> tempMap;
            for (size_t i = 0; i < size(); ++i)
            {
                tempMap.insert (at (i).toStdPair());
            }
            return std::move (tempMap);
        }

        /**
        * User convenience function to allow conversion to std::map
        * which might be used if the element is of type const
        *
        * \return std::map containing the items of the Vector<pairs>
        */
        template<typename X, typename Y>
        static DllExport std::map<X, Y> toStdMap (const Vector< Pair<X, Y> > &v)
        {
            std::map<X, Y> tempMap;
            for (size_t i = 0; i < v.size(); ++i)
            {
                tempMap.emplace (v.at (i).first, v.at (i).second);
            }
            return std::move (tempMap);
        }

    private:
        template<typename U>
        friend std::ostream &operator<< (std::ostream &os, const Vector<U> &v);

        template<typename U>
        friend void classswap (Vector<U> &first, Vector<U> &second);

        inline void freeAllocation();

        std::shared_ptr<V_TYPE> m_data;
        size_t m_allocationSize;
        size_t m_actualSize;
    };

    template<typename T>
    void classswap (Vector<T> &first, Vector<T> &second)
    {
        std::swap (first.m_actualSize, second.m_actualSize);
        std::swap (first.m_allocationSize, second.m_allocationSize);
        std::swap (first.m_data, second.m_data);
    }

    //! Template implementation
    template<typename T>
    Vector<T>::Vector() :
        m_data (nullptr),
        m_allocationSize (0),
        m_actualSize (0)
    { }

    template<typename T>
    Vector<T>::Vector (size_t allocationSize) :
        m_data (std::shared_ptr<V_TYPE> (new V_TYPE[sizeof (T) * allocationSize], std::default_delete<V_TYPE[]>())),
        m_allocationSize (allocationSize),
        m_actualSize (0)
    { }

    template<class T>
    Vector<T>::Vector (const Vector<T> &v) :
        m_data (std::shared_ptr<V_TYPE> (new V_TYPE[sizeof (T) * v.m_allocationSize], std::default_delete<V_TYPE[]>())),
        m_allocationSize (v.m_allocationSize),
        m_actualSize (v.m_actualSize)
    {
        for (size_t i = 0; i < m_actualSize; ++i)
        {
            new (m_data.get() + sizeof (T) * i) T (v[i]);
        }
    }

    template<class T>
    Vector<T>::Vector (const std::vector<T> &v) :
        m_data (std::shared_ptr<V_TYPE> (new V_TYPE[sizeof (T) * v.size()], std::default_delete<V_TYPE[]>())),
        m_allocationSize (v.size()),
        m_actualSize (v.size())
    {
        for (size_t i = 0; i < m_actualSize; ++i)
        {
            new (m_data.get() + sizeof (T) * i) T (v[i]);
        }
    }

    template<typename T>
    Vector<T>::Vector (Vector<T> &&v) :
        Vector<T>()
    {
        classswap (*this, v);
    }

    template<typename T>
    Vector<T>::Vector (const std::initializer_list<T> &list) :
        m_data (nullptr),
        m_allocationSize (0),
        m_actualSize (0)
    {
        for (auto itm : list)
        {
            push_back (itm);
        }
    }

    template<class T>
    Vector<T>::~Vector()
    {
        clear();
    }

    template<typename T>
    T *Vector<T>::data()
    {
        return (reinterpret_cast<T *> (m_data.get()));
    }

    template<typename T>
    const T *Vector<T>::data() const
    {
        return (reinterpret_cast<T *> (m_data.get()));
    }

    template<typename T>
    size_t Vector<T>::size() const
    {
        return m_actualSize;
    }

    template<typename T>
    bool Vector<T>::empty() const
    {
        return (m_actualSize == 0);
    }

    template<typename T>
    size_t Vector<T>::capacity() const
    {
        return m_allocationSize;
    }

    template<class T>
    T &Vector<T>::front()
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return (reinterpret_cast<T *> (m_data.get()) [0]);
    }

    template<class T>
    const T &Vector<T>::front() const
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return (reinterpret_cast<T *> (m_data.get()) [0]);
    }

    template<class T>
    T &Vector<T>::back()
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return (reinterpret_cast<T *> (m_data.get()) [m_actualSize - 1]);
    }

    template<class T>
    const T &Vector<T>::back() const
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return (reinterpret_cast<T *> (m_data.get()) [m_actualSize - 1]);
    }

    template<class T>
    void Vector<T>::pop_back()
    {
        if (m_actualSize > 0)
        {
            (reinterpret_cast<T *> (m_data.get()) [m_actualSize - 1]).~T();
            --m_actualSize;

            if (m_actualSize == 0)
            {
                freeAllocation();
            }
        }
    }

    template<class T>
    void Vector<T>::push_back (const T &v)
    {
        if (m_actualSize >= m_allocationSize)
        {
            reserve (m_allocationSize == 0 ? 1 : (m_allocationSize * 2));
        }
        new (m_data.get() + sizeof (T) * m_actualSize) T (v);
        m_actualSize++;
    }

    template<class T>
    void Vector<T>::push_back (T &&v)
    {
        if (m_actualSize >= m_allocationSize)
        {
            reserve (m_allocationSize == 0 ? 1 : (m_allocationSize * 2));
        }
        new (m_data.get() + sizeof (T) * m_actualSize) T (std::move (v));
        m_actualSize++;
    }

    template<class T>
    T &Vector<T>::operator[] (size_t index)
    {
        return (reinterpret_cast<T *> (m_data.get()) [index]);
    }

    template<class T>
    const T &Vector<T>::operator[] (size_t index) const
    {
        return (reinterpret_cast<T *> (m_data.get()) [index]);
    }

    template<class T>
    T &Vector<T>::at (size_t index)
    {
        if (index < m_actualSize)
        {
            return operator[] (index);
        }
        throw std::out_of_range ("index out of range");
    }

    template<class T>
    const T &Vector<T>::at (size_t index) const
    {
        if (index < m_actualSize)
        {
            return operator[] (index);
        }
        throw std::out_of_range ("index out of range");
    }

    template<typename T>
    Vector<T> &Vector<T>::operator= (Vector<T> v)
    {
        if (this != &v)
        {
            classswap (*this, v);
        }
        return *this;
    }

    template<typename T>
    Vector<T> &Vector<T>::operator= (const std::vector<T> &v)
    {
        // Create a non-const copy, so we can swap elements
        Vector<T> copy (v);
        classswap (*this, copy);
        return *this;
    }

    template<typename T>
    bool Vector<T>::operator== (const std::vector<T> &v) const
    {
        if (size() != v.size())
        {
            return false;
        }

        for (size_t i = 0; i < v.size(); ++i)
        {
            if (at (i) != v.at (i))
            {
                return false;
            }
        }

        return true;
    }

    template<typename T>
    bool Vector<T>::operator== (const Vector<T> &v) const
    {
        if (size() != v.size())
        {
            return false;
        }

        for (size_t i = 0; i < v.size(); ++i)
        {
            if (at (i) != v.at (i))
            {
                return false;
            }
        }

        return true;
    }

    template<typename T>
    bool Vector<T>::operator!= (const std::vector<T> &v) const
    {
        return !operator== (v);
    }

    template<typename T>
    bool Vector<T>::operator!= (const Vector<T> &v) const
    {
        return !operator== (v);
    }

    template <class T>
    inline void Vector<T>::freeAllocation()
    {
        if (m_data != nullptr)
        {
            m_data.reset();
            m_data = nullptr;
            m_allocationSize = 0;
        }
    }

    template <class T>
    void Vector<T>::clear()
    {
        while (m_actualSize)
        {
            pop_back();
        }

        m_actualSize = 0;
    }

    template<class T>
    void Vector<T>::reserve (size_t capacity)
    {
        if (capacity > m_allocationSize)
        {
            if (m_data == nullptr)
            {
                m_actualSize = 0;
                m_allocationSize = 0;
            }

            std::shared_ptr<V_TYPE> newBuffer (new V_TYPE[sizeof (T) * capacity], std::default_delete<V_TYPE[]>());
            for (size_t i = 0; i < m_actualSize; ++i)
            {
                new (newBuffer.get() + sizeof (T) * i) T (std::move (operator[] (i)));
            }

            m_allocationSize = capacity;

            m_data.reset();
            m_data = newBuffer;
        }
    }

    template<class T>
    void Vector<T>::resize (size_t newSize)
    {
        if (newSize != m_actualSize)
        {
            if (m_data == nullptr)
            {
                m_actualSize = 0;
                m_allocationSize = 0;
            }

            std::shared_ptr<V_TYPE> newBuffer (new V_TYPE[sizeof (T) * newSize], std::default_delete<V_TYPE[]>());
            for (size_t i = 0; i < newSize; ++i)
            {
                if (i < m_actualSize)
                {
                    new (newBuffer.get() + sizeof (T) * i) T (std::move (operator[] (i)));
                }
                else
                {
                    new (newBuffer.get() + sizeof (T) * i) T();
                }
            }

            m_actualSize = newSize;
            m_allocationSize = newSize;

            m_data.reset();
            m_data = newBuffer;
        }
    }

    template<class T>
    void Vector<T>::resize (size_t newSize, T initVal)
    {
        if (newSize != m_actualSize)
        {
            if (m_data == nullptr)
            {
                m_actualSize = 0;
                m_allocationSize = 0;
            }

            std::shared_ptr<V_TYPE> newBuffer (new V_TYPE[sizeof (T) * newSize], std::default_delete<V_TYPE[]>());
            for (size_t i = 0; i < newSize; ++i)
            {
                if (i < m_actualSize)
                {
                    new (newBuffer.get() + sizeof (T) * i) T (std::move (operator[] (i)));
                }
                else
                {
                    new (newBuffer.get() + sizeof (T) * i) T (initVal);
                }
            }

            m_actualSize = newSize;
            m_allocationSize = newSize;

            m_data.reset();
            m_data = newBuffer;
        }
    }

    template<class T>
    void Vector<T>::shrink_to_fit()
    {
        if (m_data == nullptr)
        {
            m_actualSize = 0;
            m_allocationSize = 0;
        }

        std::shared_ptr<V_TYPE> newBuffer (new V_TYPE[sizeof (T) * m_actualSize], std::default_delete<V_TYPE[]>());
        for (size_t i = 0; i < m_actualSize; ++i)
        {
            new (newBuffer.get() + sizeof (T) * i) T (std::move (operator[] (i)));
        }

        m_allocationSize = m_actualSize;

        m_data.reset();
        m_data = newBuffer;
    }

    template<class T>
    std::vector<T> Vector<T>::toStdVector()
    {
        return std::vector<T>(begin(), end());
    }

    template<class T>
    const std::vector<T> Vector<T>::toStdVector() const
    {
        return std::vector<T>(begin(), end());
    }

    template<class T>
    std::vector<T> Vector<T>::toStdVector (const Vector<T> &v)
    {
        return std::vector<T>(v.begin(), v.end());
    }

    template<class T>
    Vector<T> Vector<T>::fromStdVector (const std::vector<T> &v)
    {
        return Vector<T> (v);
    }

    template<typename T>
    std::ostream &operator<< (std::ostream &os, const Vector<T> &v)
    {
        os << "{ ";
        for (size_t i = 0; i < v.size(); ++i)
        {
            os << v.at (i);
            if (i + 1 < v.size())
            {
                os << ", ";
            }
        }
        os << " }";
        endl (os);
        return os;
    }
}

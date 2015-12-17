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
#include <cstring>
#include <string>
#include <sstream>
#include <stdexcept>
#include <memory>
#include <stdint.h>

namespace royale
{
    template<typename T>
    DllExport typename std::enable_if<
        std::is_same<T, wchar_t>::value,
        size_t
    >::type royale_strlen(const wchar_t* string)
    {
        return wcslen(string);
    }

    template<typename T>
    DllExport typename std::enable_if<
        std::is_same<T, char>::value,
        size_t
    >::type royale_strlen(const char* string)
    {
        return strlen(string);
    }


    template<class T>   // evaluates for more types of characters (UTF8/UTF16 and wide chars)
    class basicString
    {
    public:
        /**
        * Iterator definitions
        */
        typedef royale_iterator<std::random_access_iterator_tag, T> iterator;
        typedef royale_const_iterator<std::random_access_iterator_tag, T> const_iterator;
        typedef royale_reverse_iterator<std::random_access_iterator_tag, T> reverse_iterator;
        typedef royale_const_reverse_iterator<std::random_access_iterator_tag, T> const_reverse_iterator;

        /**
        * Constructor
        *
        * General constructor, which does not allocate memory and sets everything to it's default
        */
        DllExport basicString<T>();

        /**
        * Copy-Constructor for royale compliant string
        *
        * Copy constructor which allows creation of a royale compliant string from
        * another royale compliant string - (NOTE: performs a deep copy!)
        *
        * \param str The royale vector which's memory shall be copied
        */
        DllExport basicString<T> (const basicString<T> &str);

        /**
        * Copy-Constructor for STL compliant string (std::string)
        *
        * Copy constructor which allows creation of a royale compliant string from
        * a STL compliant string - (NOTE: performs a deep copy!)
        *
        * \param str The STL string to copy
        */
        DllExport basicString<T> (const std::basic_string<T> &str);

        /**
        * Move-Constructor for royale compliant string
        *
        * Constructor which allows creation of a royale compliant string by moving memory
        * (NOTE: performs a shallow copy!)
        *
        * \param str The royale string which's memory shall be moved
        */
        DllExport basicString<T> (basicString<T> &&str);

        /**
        * Constructor which allows passing a royale compliant string and add it at
        * a given position - the caller might also limit the length that shall be copied
        * (NOTE: performs a deep copy!)
        *
        * If the len would exceed the length of str then all bytes till the end of
        * str are copied to the new string object.
        *
        * \param str The royale string which shall be copied
        * \param pos The position where to start to copy bytes
        * \param len The length/amount of bytes that shall be copied
        */
        DllExport basicString<T> (const basicString<T> &str, size_t pos, size_t len = npos);

        /**
        * Constructor which allows passing a STL compliant string and add it at
        * a given position - the caller might also limit the length that shall be copied
        * (NOTE: performs a deep copy!)
        *
        * If the len would exceed the length of str then all bytes till the end of
        * str are copied to the new string object.
        *
        * \param str The STL compliant string which shall be copied
        * \param pos The position where to start to copy bytes
        * \param len The length/amount of bytes that shall be copied
        */
        DllExport basicString<T> (const std::basic_string<T> &str, size_t pos, size_t len = npos);

        /**
        * Constructor which allows passing a C-Style string
        * to create a royale compliant string
        * (NOTE: performs a deep copy!)
        *
        * \param s The C-Style compliant string which shall be copied
        */
        DllExport basicString<T> (const T *s);

        /**
        * Constructor which allows passing a C-Style string
        * to create a royale compliant string by copying the first n bytes
        * (NOTE: performs a deep copy!)
        *
        * If n is bigger than the length of s, then n is allocated and the
        * length of s is  used for copying - this results in a string
        * with unused allocated space and the value of where s points to.
        *
        * If n is smaller than the length of s, then n is allocated and
        * only n bytes from s are copied to the string object.
        *
        * \param s The C-Style compliant string which shall be copied
        * \param n The number of bytes to copy
        */
        DllExport basicString<T> (const T *s, size_t n);

        /**
        * Constructor which allows creating a royale compliant string with
        * n slots which are initialized by character c.
        *
        * \param n The number of slots that shall be reserved
        * \param c The character which is used for the slot's initialization
        */
        DllExport basicString<T> (size_t n, T c);

        /**
        * Initializer list initialization to initialize a string
        *
        * \param list The list of values
        */
        DllExport basicString<T> (const std::initializer_list<T> &list);

        /**
        * Destructor
        *
        * Clears the string's allocated memory by performing deletion
        */
        DllExport virtual ~basicString<T>();

        /**
        * Returns the actual size/length of the string (this is the used amount of
        * slots in the allocated area)
        *
        * \return size_t The amount of the actual used memory slots within the string
        */
        DllExport size_t size() const;

        /**
        * Returns the actual size/length of the string (this is the used amount of
        * slots in the allocated area)
        *
        * \return size_t The amount of the actual used memory slots within the string
        */
        DllExport size_t length() const;

        /**
        * Checks if the string is empty
        *
        * \return bool Returns true if the string is empty - otherwise false
        */
        DllExport bool empty() const;

        /**
        * Returns the amount of allocated slots which are maintained by the vector
        * These allocated slots might be used or unused (refer to size() for checking the size()
        * itself)
        *
        * \return size_t The amount of allocated slots for the element type which is bound to the string
        */
        DllExport size_t capacity() const;

        /**
        * Returns a direct pointer to the memory array used internally by the string to store its owned elements.
        *
        * Because elements in the string are guaranteed to be stored in contiguous storage locations in the same order as
        * represented by the string, the pointer retrieved can be offset to access any element in the array.
        *
        * \return T* A pointer to the first element in the array used internally by the string.
        */
        DllExport T *data();

        /**
        * Returns a direct pointer to the memory array used internally by the string to store its owned elements.
        *
        * Because elements in the string are guaranteed to be stored in contiguous storage locations in the same order as
        * represented by the string, the pointer retrieved can be offset to access any element in the array.
        *
        * \return T* A pointer to the first element in the array used internally by the string.
        */
        DllExport const T *data() const;

        /**
        * Returns a reference to the first element in the string.
        * Calling this function on an empty string will result in a std::out_of_range exception.
        *
        * \return T& A reference to the first element in the string.
        * \throws std::out_of_range Exception if the string is empty
        */
        DllExport T &front();
        DllExport const T &front() const;

        /**
        * Returns an iterator to the first position
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator An iterator to the begin of the string.
        */
        DllExport iterator begin()
        {
            return iterator (m_data.get());
        }

        DllExport const_iterator begin() const
        {
            return const_iterator (m_data.get());
        }

        /**
        * Returns an iterator to the last position
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator An iterator to the end of the string.
        */
        DllExport iterator end()
        {
            return iterator (m_data.get() + m_actualSize);
        }

        DllExport const_iterator end() const
        {
            return const_iterator (m_data.get() + m_actualSize);
        }

        /**
        * Returns an iterator to the reverse begin (last position)
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator An iterator to the reverse begin of the string
        */
        DllExport reverse_iterator rbegin()
        {
            return reverse_iterator (m_data.get() + m_actualSize - 1);
        }

        DllExport const_reverse_iterator rbegin() const
        {
            return const_reverse_iterator (m_data.get() + m_actualSize - 1);
        }

        /**
        * Returns an iterator to the reverse end (first position)
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator An iterator to the reverse end of the string
        */
        DllExport reverse_iterator rend()
        {
            return reverse_iterator (m_data.get() - 1);
        }

        DllExport const_reverse_iterator rend() const
        {
            return const_reverse_iterator (m_data.get() - 1);
        }

        /**
        * Returns a constant iterator to the begin of the string (first position)
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator A constant iterator to the begin of the string
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
        * Returns a constant iterator to the end of the string (last position)
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator A constant iterator to the end of the string
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
        * Returns a constant iterator to the reverse begin of the string (last position)
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator A constant iterator to the reverse begin of a string
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
        * Returns a constant iterator to the reverse end of the string (first position)
        * Calling this function on an empty string will result in undefined behavior
        *
        * \return iterator A constant iterator to the reverse end of a string
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
        * Returns a reference to the last element in the string.
        * Calling this function on an empty vector will result in a std::out_of_range exception.
        *
        * \return T& A reference to the last element in the string.
        * \throws std::out_of_range Exception if the string is empty
        */
        DllExport T &back();
        DllExport const T &back() const;

        /**
        * Adds a new element at the end of the string, after its current last element.
        * The content of str is copied to the new element (NOTE: a deep copy is performed!).
        * This effectively increases the container size, which causes an automatic reallocation
        * of the allocated storage space if -and only if- the new string size surpasses the current string capacity.
        */
        DllExport void push_back (const std::basic_string<T> &str);
        DllExport void push_back (const basicString<T> &str);
        DllExport void push_back (const std::basic_string<T> &str, size_t subpos, size_t sublen);
        DllExport void push_back (const basicString<T> &str, size_t subpos, size_t sublen);
        DllExport void push_back (const T *str);
        DllExport void push_back (const T *str, size_t n);
        DllExport void push_back (size_t n, T c);
        DllExport void push_back (const T c);

        /**
        * Adds a new element at the end of the string, after its current last element
        * and returns the actual string element.
        * The content of str is copied to the new element (NOTE: a deep copy is performed!).
        * This effectively increases the container size, which causes an automatic reallocation
        * of the allocated storage space if -and only if- the new string size surpasses the current string capacity.
        */
        DllExport basicString<T> &append (const std::basic_string<T> &str);
        DllExport basicString<T> &append (const basicString<T> &str);
        DllExport basicString<T> &append (const std::basic_string<T> &str, size_t subpos, size_t sublen);
        DllExport basicString<T> &append (const basicString<T> &str, size_t subpos, size_t sublen);
        DllExport basicString<T> &append (const T *str);
        DllExport basicString<T> &append (const T *str, size_t n);
        DllExport basicString<T> &append (size_t n, T c);
        DllExport basicString<T> &append (const T c);

        /**
        * Adds a new element at the end of the string, after its current last element
        * and returns the actual string element.
        * The content of str is copied to the new element (NOTE: a deep copy is performed!).
        * This effectively increases the container size, which causes an automatic reallocation
        * of the allocated storage space if -and only if- the new string size surpasses the current string capacity.
        */
        DllExport basicString<T> &operator+= (const std::basic_string<T> &str);
        DllExport basicString<T> &operator+= (const basicString<T> &str);
        DllExport basicString<T> &operator+= (const T *s);
        DllExport basicString<T> &operator+= (const T s);

        /**
        * Adds two stirngs and returns the result (the second string is appended)
        * The content of str is copied to the newly created element (NOTE: a deep copy is performed!).
        * This effectively increases the container size, which causes an automatic reallocation
        * of the allocated storage space if -and only if- the new string size surpasses the current string capacity.
        */
        DllExport basicString<T> operator+ (const std::basic_string<T> &str) const;
        DllExport basicString<T> operator+ (const basicString<T> &str) const;
        DllExport basicString<T> operator+ (const T *s) const;
        DllExport basicString<T> operator+ (const T s) const;

        /**
        * Removes the last element in the string, effectively reducing the container size by one.
        * The last element is destroyed by calling it's destructor; the size is reduced by one BUT
        * there is no reallocation performed to resize the string to it's contents or to reduce the
        * the string capacity by one.
        * The allocated space remains the same.
        */
        DllExport void pop_back();

        /**
        * Access an element
        *
        * Returns a reference to the element at position index in the string container.
        * A similar member function, at(), has the same behavior as this operator function,
        * except that at() is bound-checked and signals if the requested position is
        * out of range by throwing an out_of_range exception.
        *
        * \param index The index to access within the string's storage
        * \return T& The Reference to the elements at the specified position in the string
        */
        DllExport T &operator[] (size_t index);

        /**
        * Access an element
        *
        * Returns a reference to the element at position index in the string container.
        * A similar member function, at(), has the same behavior as this operator function,
        * except that at() is bound-checked and signals if the requested position is
        * out of range by throwing an out_of_range exception.
        *
        * \param index The index to access within the string's storage
        * \return T& The Reference to the elements at the specified position in the string
        */
        DllExport const T &operator[] (size_t index) const;

        /**
        * Access an element
        *
        * The function automatically checks whether index is within the bounds
        * of valid elements in the string, throwing an out_of_range exception
        * if it is not (i.e., if index is greater or equal than its size).
        * This is in contrast with member operator[], that does not check against bounds.
        *
        * \param index The index to access within the string's storage
        * \return T& The Reference to the elements at the specified position in the string
        */
        DllExport T &at (size_t index);

        /**
        * Access an element
        *
        * The function automatically checks whether index is within the bounds
        * of valid elements in the string, throwing an out_of_range exception
        * if it is not (i.e., if index is greater or equal than its size).
        * This is in contrast with member operator[], that does not check against bounds.
        *
        * \param index The index to access within the string's storage
        * \return T& The Reference to the elements at the specified position in the string
        */
        DllExport const T &at (size_t index) const;

        /**
        * Assign another royale compliant string
        *
        * Assigns new contents to the container, replacing its current contents,
        * and modifying its size accordingly.
        * This method copies all elements held by str into the container
        *
        * \param str A string of the same storage type
        * \return basicString<T>& Returns *this
        */
        DllExport basicString<T> &operator= (const basicString<T> &str);

        /**
        * Move elements to another royale compliant string
        *
        * Moves the elements of str into the container.
        * The source container is reset to it's initial state which
        * causes the allocation size to be reset (the allocated memory is moved)
        * and the size counter to be reset.
        * The data pointer is set to null.
        *
        * Leaves the source object in a valid/initial state allowing
        * somebody to reuse it for other purposes/data - like after
        * executing clear())
        *
        * \param str A string object of the same type (i.e., with the same template parameters, T and Alloc).
        * \return basicString<T>& Returns *this
        */
        DllExport basicString<T> &operator= (basicString<T> &&str);

        /**
        * Assign the contents of an STL compliant string container by
        * replacing container's current contents if necessary
        * and modifying its size accordingly.
        * This method copies all elements held by str into the container
        *
        * \param str An STL compliant string of the same storage type
        * \return basicString<T>& Returns *this
        */
        DllExport basicString<T> &operator= (const std::basic_string<T> &str);

        /**
        * Assign the contents of an C-Style compliant string container by
        * replacing container's current contents if necessary
        * and modifying its size accordingly.
        * This method copies all elements held by str into the container
        *
        * \param str An C-Style compliant string of the same storage type
        * \return basicString<T>& Returns *this
        */
        DllExport basicString<T> &operator= (const T *str);

        /**
        * Assign a character to a royale compliant string container by
        * replacing container's current contents and modifying its size accordingly.
        *
        * \param str The character to assign to the royale compliant string
        * \return basicString<T>& Returns *this
        */
        DllExport basicString<T> &operator= (const T str);

        /**
        * Checks equality with an STL compliant vector
        *
        * \param str An STL compliant string of the same storage type
        * \return bool Returns true if they are equal and false is they are not
        */
        DllExport bool operator== (const std::basic_string<T> &str) const;

        /**
        * Checks equality with a royale compliant string
        *
        * \param str An royale compliant string of the same storage type
        * \return bool Returns true if they are equal and false is they are not
        */
        DllExport bool operator== (const basicString<T> &str) const;

        /**
        * Checks equality with a C-Style string
        *
        * \param str An C-Style string of the same storage type
        * \return bool Returns true if they are equal and false is they are not
        */
        DllExport bool operator== (const T *str) const;

        /**
        * Checks unequality with an STL compliant string
        *
        * \param str An STL compliant string of the same storage type
        * \return bool Returns false if they are equal and true is they are not
        */
        DllExport bool operator!= (const std::basic_string<T> &str) const;

        /**
        * Checks unequality with a royale compliant string
        *
        * \param str An royale compliant string of the same storage type
        * \return bool Returns false if they are equal and true is they are not
        */
        DllExport bool operator!= (const basicString<T> &str) const;

        /**
        * Checks unequality with a C-Style string
        *
        * \param str A C-style compliant string of the same storage type
        * \return bool Returns false if they are equal and true is they are not
        */
        DllExport bool operator!= (const T *str) const;

        /**
        * Removes all elements from the string (which are destroyed), leaving the container with a size of 0.
        *
        * A reallocation is not performed and the string's capacity is destroyed (everything is freed).
        */
        DllExport void clear();

        /**
        * Modifies the string to the given allocation size and allocates the buffers (it may shrink)
        *
        * Creates any amount of elements (allocates the memory already)
        * and moves the existing elements to these slots; afterwards the old space is dumped.
        *
        * If the given newSize is smaller than the already used slots, the string will shrink.
        * This means that all elements which are not covered within this capacity (the last ones) will be
        * deleted.
        *
        * \param newSize The amount of slots to remain in the string (might shrink or enlarge the string)
        */
        DllExport void resize (size_t newSize);

        /**
        * Extends the string to a higher allocation size and allocates the buffers
        *
        * Reserves any amount of free allocation slots (allocates the memory already) to be later
        * used for the element-types bound to the given string vector.
        *
        * If the given size to reserve is smaller than the already reserved space, then the function
        * return immediately; otherwise the necessary memory allocation is performed and the size is
        * extended to "size"
        *
        * \param size The size (number of element-types) of elements that should be allocated.
        */
        DllExport void reserve (size_t size);

        /**
        * Shrinks the string's allocation to it's size
        *
        * Changes the size of the allocated buffer to the string's size
        * this may result in freeing unneeded memory allocation.
        */
        DllExport void shrink_to_fit();

        /**
        * Converts the given std::basic_string (STL) to the string type used by the royale API
        *
        * \param str The STL string which should be converted to the royale API vector format
        * \return The royale API compliant string format
        */
        static DllExport basicString<T> fromStdString (const std::basic_string<T> &str);

        /**
        * Converts the given C-Style array to the string type used by the royale API
        *
        * \param str The C-Style array which should be converted to the royale API vector format
        * \return The royale API compliant string format
        */
        static DllExport basicString<T> fromCArray (const T *str);

        /**
        * Convert to String from any Type
        */
        template<typename... Dummy, typename U, typename Type = T>
        static DllExport typename std::enable_if<
            std::is_same<Type, char>::value,
            basicString<Type>
        >::type fromAny(const U value)
        {
            std::ostringstream os;
            os << value;
            std::string stdString(os.str());
            return basicString<T>(stdString);
        }

        /**
        * Convert to WString from any Type
        */
        template<typename... Dummy, typename U, typename Type = T>
        static DllExport typename std::enable_if<
            std::is_same<Type, wchar_t>::value,
            basicString<Type>
        >::type fromAny(const U value)
        {
            std::wstringstream os;
            os << value;
            std::wstring stdString(os.str());
            return basicString<Type>(stdString);
        }

        /**
        * Convert to String from Int
        */
        static DllExport basicString<T> fromInt (int value)
        {
            return basicString<T>::fromAny<int> (value);
        }

        /**
        * Convert to String from unsigned Int
        */
        static DllExport basicString<T> fromUInt (unsigned int value)
        {
            return basicString<T>::fromAny<unsigned int> (value);
        }

        /**
        * Converts the royale string object to a C-Style array
        *
        * \return T* The C-Style Array
        */
        DllExport T *c_str();
        DllExport const T *c_str() const;

        /**
        * User convenience function to allow conversion to std::basic_string
        * which might be used outside the library by the application for further processing
        *
        * \return std::basic_string containing the items of the
        */
        DllExport std::basic_string<T> toStdString();

        /**
        * User convenience function to allow conversion to std::basic_string
        * which might be used if the royale compliant string is a const
        *
        * \return std::basic_string containing the items of the
        */
        static DllExport std::basic_string<T> toStdString (const basicString<T> &str);

        /**
        * Returns the max_size of the string
        *
        * \return size_t The maximum length of the string
        */
        size_t max_size() const;

    private:
        const int NULL_BYTE_LENGTH = 1;

        static const char EOS = '\0';
        static const size_t npos = -1;

        inline void preReserve (const std::basic_string<T> &str, size_t sublen = 0);
        inline void preReserve (const basicString<T> &str, size_t sublen = 0);
        inline void preReserve (const T *s, size_t len = 0);
        inline void preReserve (size_t len);

        template<typename U>
        friend std::ostream &operator<< (std::ostream &os, const basicString<U> &str);

        size_t m_allocationSize;
        size_t m_actualSize;
        std::shared_ptr<T> m_data;
    };

    //! Template implementation
    template<typename T>
    basicString<T>::basicString() :
        m_allocationSize (0),
        m_actualSize (0),
        m_data (nullptr)
    { }

    template<typename T>
    basicString<T>::basicString (const basicString<T> &str)
        : basicString()
    {
        if (str.m_actualSize)
        {
            m_actualSize = str.m_actualSize;
            m_allocationSize = str.m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_allocationSize; ++i)
            {
                m_data.get() [i] = str.m_data.get() [i];
            }
        }
    }


    template<typename T>
    basicString<T>::basicString (const std::basic_string<T> &str)
        : basicString()
    {
        if (str.length())
        {
            m_actualSize = str.length();
            m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_allocationSize; ++i)
            {
                m_data.get() [i] = str[i];
            }
        }
    }

    template<typename T>
    basicString<T>::basicString (const basicString<T> &str, size_t pos, size_t len)
        : basicString()
    {
        if (str.length())
        {
            size_t endPos = ( (len + pos) > str.length() ? str.length() : (len + pos));

            m_actualSize = endPos - pos;
            m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_actualSize; ++i)
            {
                m_data.get() [i] = str.m_data.get() [pos + i];
            }
            m_data.get() [m_actualSize] = EOS;
        }
    }

    template<typename T>
    basicString<T>::basicString (const std::basic_string<T> &str, size_t pos, size_t len)
        : basicString()
    {
        if (str.length())
        {
            size_t endPos = ( (len + pos) > str.length() ? str.length() : (len + pos));

            m_actualSize = endPos - pos;
            m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_actualSize; ++i)
            {
                m_data.get() [i] = str[pos + i];
            }
            m_data.get() [m_actualSize] = EOS;
        }
    }

    template<typename T>
    basicString<T>::basicString (const T *s)
        : basicString()
    {
        m_actualSize = royale_strlen<T>(s);
        if (m_actualSize)
        {
            m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_actualSize; ++i)
            {
                m_data.get() [i] = s[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }
    }

    template<typename T>
    basicString<T>::basicString (const T *s, size_t n)
        : basicString()
    {
        if (n > 0)
        {
            m_actualSize = (royale_strlen<T>(s) < n ? royale_strlen<T>(s) : n);
            m_allocationSize = n + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            size_t i = 0;
            for (i = 0; i < m_actualSize; ++i)
            {
                m_data.get() [i] = s[i];
            }
            m_data.get() [i] = EOS;
        }
    }

    template<typename T>
    basicString<T>::basicString (size_t n, T c)
        : basicString()
    {
        if (n > 0)
        {
            m_actualSize = n;
            m_allocationSize = n + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            size_t i;
            for (i = 0; i < n; ++i)
            {
                m_data.get() [i] = c;
            }
            m_data.get() [i] = EOS;
        }
    }

    template<typename T>
    basicString<T>::basicString (const std::initializer_list<T> &list) :
        m_allocationSize (0),
        m_actualSize (0),
        m_data (nullptr)
    {
        for (auto itm : list)
        {
            append (itm);
        }
    }

    template<typename T>
    basicString<T>::basicString (basicString<T> &&str) :
        m_allocationSize (0),
        m_actualSize (0),
        m_data (nullptr)
    {
        *this = std::move (str);
    }

    template<class T>
    basicString<T>::~basicString()
    { }

    template<typename T>
    T *basicString<T>::data()
    {
        return m_data.get();
    }

    template<typename T>
    const T *basicString<T>::data() const
    {
        return m_data.get();
    }

    template<typename T>
    size_t basicString<T>::size() const
    {
        return m_actualSize;
    }

    template<typename T>
    size_t basicString<T>::length() const
    {
        return m_actualSize;
    }

    template<typename T>
    bool basicString<T>::empty() const
    {
        return (m_actualSize == 0);
    }

    template<typename T>
    size_t basicString<T>::capacity() const
    {
        return m_allocationSize;
    }

    template<class T>
    T &basicString<T>::front()
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return m_data.get() [0];
    }

    template<class T>
    const T &basicString<T>::front() const
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return m_data.get() [0];
    }

    template<class T>
    T &basicString<T>::back()
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return m_data.get() [m_actualSize - 1];
    }

    template<class T>
    const T &basicString<T>::back() const
    {
        if (m_data == nullptr || empty() == true)
        {
            throw std::out_of_range ("index out of range");
        }
        return m_data.get() [m_actualSize - 1];
    }

    template<class T>
    size_t basicString<T>::max_size() const
    {
        return npos;
    }

    template<class T>
    void basicString<T>::reserve (size_t capacity)
    {
        if (capacity > m_allocationSize)
        {
            if (m_data == nullptr)
            {
                m_actualSize = 0;
                m_allocationSize = 0;
            }

            std::shared_ptr<T> newBuffer (new T[capacity + NULL_BYTE_LENGTH], std::default_delete<T[]>());
            for (size_t i = 0; i < m_actualSize; ++i)
            {
                newBuffer.get() [i] = m_data.get() [i];
            }
            newBuffer.get() [m_actualSize] = EOS;

            m_allocationSize = capacity;
            m_data.reset();
            m_data = newBuffer;
        }
    }

    template<class T>
    void basicString<T>::resize (size_t newSize)
    {
        if (newSize != m_actualSize)
        {
            if (m_data == nullptr)
            {
                m_actualSize = 0;
                m_allocationSize = 0;
            }

            std::shared_ptr<T> newBuffer (new T[newSize + NULL_BYTE_LENGTH], std::default_delete<T[]>());
            memset (newBuffer.get(), 0, sizeof (* (newBuffer.get())));

            size_t l_Size = newSize < m_allocationSize ? newSize : m_allocationSize;
            for (size_t i = 0; i < l_Size; ++i)
            {
                newBuffer.get() [i] = m_data.get() [i];
            }
            newBuffer.get() [l_Size] = EOS;

            m_actualSize = newSize;
            m_allocationSize = newSize + NULL_BYTE_LENGTH;

            m_data.reset();
            m_data = newBuffer;
        }
    }

    template<class T>
    void basicString<T>::shrink_to_fit()
    {
        if (m_data == nullptr)
        {
            m_actualSize = 0;
            m_allocationSize = 0;
        }

        std::shared_ptr<T> newBuffer (new T[m_actualSize + NULL_BYTE_LENGTH], std::default_delete<T[]>());
        for (size_t i = 0; i < (m_actualSize + NULL_BYTE_LENGTH); ++i)
        {
            newBuffer.get() [i] = m_data.get() [i];
        }

        m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;

        m_data.reset();
        m_data = newBuffer;
    }

    template<class T>
    inline void basicString<T>::preReserve (const std::basic_string<T> &str, size_t sublen)
    {
        if (capacity() < size() + NULL_BYTE_LENGTH + ( (sublen == 0 ? str.length() : sublen)))
        {
            reserve (size_t (capacity() + (capacity() / 2) + (sublen == 0 ? str.length() : sublen)));
        }
    }

    template<class T>
    inline void basicString<T>::preReserve (const basicString<T> &str, size_t sublen)
    {
        if (capacity() < size() + NULL_BYTE_LENGTH + ( (sublen == 0 ? str.length() : sublen)))
        {
            reserve (size_t (capacity() + (capacity() / 2) + (sublen == 0 ? str.length() : sublen)));
        }
    }

    template<class T>
    inline void basicString<T>::preReserve (const T *s, size_t len)
    {
        if (capacity() < (size() + NULL_BYTE_LENGTH + (len == 0 ? royale_strlen<T>(s) : len)))
        {
            reserve(size_t(capacity() + (capacity() / 2) + (len == 0 ? royale_strlen<T>(s) : len)));
        }
    }

    template<class T>
    inline void basicString<T>::preReserve (size_t len)
    {
        if (capacity() < (size() + NULL_BYTE_LENGTH + len))
        {
            reserve (size_t (capacity() + (capacity() / 2) + len));
        }
    }

    template<class T>
    basicString<T> &basicString<T>::append (const std::basic_string<T> &str)
    {
        preReserve (str);

        for (size_t i = 0; i < str.length(); ++i)
        {
            m_data.get() [m_actualSize++] = str[i];
        }
        m_data.get() [m_actualSize] = EOS;

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (const basicString<T> &str)
    {
        if (str.length())
        {
            preReserve (str);

            for (size_t i = 0; i < str.length(); ++i)
            {
                m_data.get() [m_actualSize++] = str[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (const std::basic_string<T> &str, size_t subpos, size_t sublen)
    {
        size_t endPos = ( (subpos + sublen) < str.length() ? (subpos + sublen) : str.length());

        if (endPos - subpos > 0)
        {
            preReserve (str, endPos - subpos);

            for (size_t i = subpos; i < endPos; ++i)
            {
                m_data.get() [m_actualSize++] = str[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (const basicString<T> &str, size_t subpos, size_t sublen)
    {
        size_t endPos = ( (subpos + sublen) < str.length() ? (subpos + sublen) : str.length());

        if (endPos - subpos > 0)
        {
            preReserve (str, endPos - subpos);

            for (size_t i = subpos; i < endPos; ++i)
            {
                m_data.get() [m_actualSize++] = str[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (const T *str)
    {
        if (royale_strlen<T>(str) > 0)
        {
            preReserve(royale_strlen<T>(str));

            for (size_t i = 0; i < royale_strlen<T>(str); ++i)
            {
                m_data.get() [m_actualSize++] = str[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (const T *str, size_t n)
    {
        if (!n)
        {
            return *this;
        }

        size_t copy_n = (n > royale_strlen<T>(str) ? royale_strlen<T>(str) : n);
        if (copy_n)
        {
            preReserve (str, copy_n);

            size_t copy_n = (n > royale_strlen<T>(str) ? royale_strlen<T>(str) : n);
            for (size_t i = 0; i < copy_n; ++i)
            {
                m_data.get() [m_actualSize++] = str[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (size_t n, T c)
    {
        if (n)
        {
            preReserve (n);

            for (size_t i = 0; i < n; ++i)
            {
                m_data.get() [m_actualSize++] = c;
            }
            m_data.get() [m_actualSize] = EOS;
        }

        return *this;
    }

    template<class T>
    basicString<T> &basicString<T>::append (const T c)
    {
        preReserve (m_actualSize + 1);

        m_data.get() [m_actualSize++] = c;
        m_data.get() [m_actualSize] = EOS;

        return *this;
    }

    template<typename T>
    basicString<T> &basicString<T>::operator+= (const std::basic_string<T> &str)
    {
        return append (str);
    }

    template<typename T>
    basicString<T> &basicString<T>::operator+= (const basicString<T> &str)
    {
        return append (str);
    }

    template<typename T>
    basicString<T> &basicString<T>::operator+= (const T *s)
    {
        return append (s);
    }

    template<typename T>
    basicString<T> &basicString<T>::operator+= (const T s)
    {
        return append (s);
    }

    template<typename T>
    basicString<T> basicString<T>::operator+ (const std::basic_string<T> &str) const
    {
        basicString<T> bString (*this);
        bString += str;
        return std::move (bString);
    }

    template<typename T>
    basicString<T> basicString<T>::operator+ (const basicString<T> &str) const
    {
        basicString<T> bString (*this);
        bString += str;
        return std::move (bString);
    }

    template<typename T>
    basicString<T> basicString<T>::operator+ (const T *s) const
    {
        basicString<T> bString (*this);
        bString += s;
        return std::move (bString);
    }

    template<typename T>
    basicString<T> basicString<T>::operator+ (const T s) const
    {
        basicString<T> bString (*this);
        bString += s;
        return std::move (bString);
    }

    template<class T>
    void basicString<T>::pop_back()
    {
        if (m_actualSize > 0)
        {
            m_actualSize--;
            m_data.get() [m_actualSize] = EOS;

            if (!m_actualSize && m_data != nullptr)
            {
                m_data.reset();
                m_data = nullptr;

                m_allocationSize = 0;
            }
        }
    }

    template<class T>
    void basicString<T>::push_back (const std::basic_string<T> &str)
    {
        append (str);
        return;
    }

    template<class T>
    void basicString<T>::push_back (const basicString<T> &str)
    {
        append (str);
        return;
    }

    template<class T>
    void basicString<T>::push_back (const std::basic_string<T> &str, size_t subpos, size_t sublen)
    {
        append (str, subpos, sublen);
        return;
    }

    template<class T>
    void basicString<T>::push_back (const basicString<T> &str, size_t subpos, size_t sublen)
    {
        append (str, subpos, sublen);
        return;
    }

    template<class T>
    void basicString<T>::push_back (const T *str)
    {
        append (str);
        return;
    }

    template<class T>
    void basicString<T>::push_back (const T *str, size_t n)
    {
        append (str, n);
        return;
    }

    template<class T>
    void basicString<T>::push_back (size_t n, T c)
    {
        append (n, c);
        return;
    }

    template<class T>
    void basicString<T>::push_back (const T c)
    {
        append (c);
        return;
    }

    template<class T>
    T &basicString<T>::at (size_t index)
    {
        if (index < (m_actualSize + NULL_BYTE_LENGTH))
        {
            return m_data.get() [index];
        }
        throw std::out_of_range ("index out of range");
    }

    template<class T>
    const T &basicString<T>::at (size_t index) const
    {
        if (index < (m_actualSize + NULL_BYTE_LENGTH))
        {
            return m_data.get() [index];
        }
        throw std::out_of_range ("index out of range");
    }

    template<class T>
    T &basicString<T>::operator[] (size_t index)
    {
        return m_data.get() [index];
    }

    template<class T>
    const T &basicString<T>::operator[] (size_t index) const
    {
        return m_data.get() [index];
    }

    template<typename T>
    basicString<T> &basicString<T>::operator= (const basicString<T> &str)
    {
        if (this != &str)
        {
            // Free the existing resource.
            m_data.reset();

            m_actualSize = str.m_actualSize;
            if (m_actualSize)
            {
                m_allocationSize = str.m_actualSize + NULL_BYTE_LENGTH;
                m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

                for (size_t i = 0; i < str.m_actualSize; ++i)
                {
                    m_data.get() [i] = str.m_data.get() [i];
                }
                m_data.get() [str.m_actualSize] = EOS;
            }
            else
            {
                m_allocationSize = 0;
                m_data = nullptr;
            }
        }
        return *this;
    }

    template<typename T>
    basicString<T> &basicString<T>::operator= (const std::basic_string<T> &str)
    {
        // Free the existing resource.
        m_data.reset();

        m_actualSize = str.length();
        if (m_actualSize)
        {
            m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_actualSize; ++i)
            {
                m_data.get() [i] = str[i];
            }
            m_data.get() [m_actualSize] = EOS;
        }
        else
        {
            m_allocationSize = 0;
            m_data = nullptr;
        }

        return *this;
    }

    template<typename T>
    basicString<T> &basicString<T>::operator= (const T *str)
    {
        // Free the existing resource.
        m_data.reset();

        m_actualSize = royale_strlen<T>(str);
        if (m_actualSize)
        {
            m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
            m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

            for (size_t i = 0; i < m_actualSize; ++i)
            {
                m_data.get() [i] = * (str + i);
            }
            m_data.get() [m_actualSize] = EOS;
        }
        else
        {
            m_allocationSize = 0;
            m_data = nullptr;
        }

        return *this;
    }

    template<typename T>
    basicString<T> &basicString<T>::operator= (const T str)
    {
        // Free the existing resource.
        m_data.reset();

        m_actualSize = 1;
        m_allocationSize = m_actualSize + NULL_BYTE_LENGTH;
        m_data = std::shared_ptr<T> (new T[m_allocationSize], std::default_delete<T[]>());

        m_data.get() [0] = str;
        m_data.get() [m_actualSize] = EOS;

        return *this;
    }

    template<typename T>
    basicString<T> &basicString<T>::operator= (basicString<T> &&str)
    {
        if (this != &str)
        {
            // Free the existing resource.
            m_data.reset();

            // Copy the data pointer and the lengths from the source object
            m_data = str.m_data;
            m_actualSize = str.m_actualSize;
            m_allocationSize = str.m_actualSize;

            // Release the data pointer from the source object
            str.m_data = nullptr;
            str.m_actualSize = 0;
            str.m_allocationSize = 0;
        }
        return *this;
    }

    template<typename T>
    bool basicString<T>::operator== (const std::basic_string<T> &str) const
    {
        if (length() != str.length())
        {
            return false;
        }

        for (size_t i = 0; i < str.length(); ++i)
        {
            if (at (i) != str[i])
            {
                return false;
            }
        }

        return true;
    }

    template<typename T>
    bool basicString<T>::operator== (const basicString<T> &str) const
    {
        if (length() != str.length())
        {
            return false;
        }

        for (size_t i = 0; i < str.length(); ++i)
        {
            if (at (i) != str.at (i))
            {
                return false;
            }
        }

        return true;
    }

    template<typename T>
    bool basicString<T>::operator== (const T *str) const
    {
        if (length() != royale_strlen<T>(str))
        {
            return false;
        }

        for (size_t i = 0; i < royale_strlen<T>(str); ++i)
        {
            if (at (i) != str[i])
            {
                return false;
            }
        }

        return true;
    }

    template<typename T>
    bool basicString<T>::operator!= (const std::basic_string<T> &str) const
    {
        return !operator== (str);
    }

    template<typename T>
    bool basicString<T>::operator!= (const basicString<T> &str) const
    {
        return !operator== (str);
    }

    template<typename T>
    bool basicString<T>::operator!= (const T *str) const
    {
        return !operator== (str);
    }

    template <class T>
    void basicString<T>::clear()
    {
        if (m_data != nullptr)
        {
            m_data.reset();
            m_data = nullptr;
            m_allocationSize = 0;
        }

        m_actualSize = 0;
    }

    template<class T>
    T *basicString<T>::c_str()
    {
        return data();
    }

    template<class T>
    const T *basicString<T>::c_str() const
    {
        return data();
    }

    template<class T>
    std::basic_string<T> basicString<T>::toStdString()
    {
        return std::basic_string<T> (c_str());
    }

    template<class T>
    std::basic_string<T> basicString<T>::toStdString (const basicString<T> &str)
    {
        return std::basic_string<T> (str.c_str());
    }

    template<class T>
    basicString<T> basicString<T>::fromStdString (const std::basic_string<T> &str)
    {
        return basicString<T> (str);
    }

    template<class T>
    basicString<T> basicString<T>::fromCArray (const T *str)
    {
        return basicString<T> (str);
    }

    template<typename T>
    std::ostream &operator<< (std::ostream &os, const basicString<T> &str)
    {
        if (str.length() > 0)
        {
            os << str.c_str();
        }
        return os;
    }

    typedef basicString<char> String;
    typedef basicString<wchar_t> WString;
}

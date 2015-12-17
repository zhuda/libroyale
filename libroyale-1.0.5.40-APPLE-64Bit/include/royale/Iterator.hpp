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

#include <iterator>
#include <cstddef>
#include <type_traits>

/* IMPORTANT */
/* The version below, enclosed by if'defs is the correct solution and works properly on Linux
   however there is a compiler bug for VS (spotted with VS2013 Update 5) which avoids the correct
   implementation for windows, however this implementation of iterator is correct as long as
   there are only iterators of type std::random_access_iterator_tag are used.

   One might think of a solution with multiple classes on top of their base iterator classes
   and inherit from different base classes based on the iterator_tag by using enable_if.
   This might intruduce lots of other problems due to the complex unflattend design, so it
   was avoided.

   Workaround: Make every iterator that you declare a specialization from these types by using
   std::random_access_iterator_tag - the correct implementation for iterator is given below,
   enclosed by #if 0/#endif.
   The workaround is below the correct version.

   RAISED COMPILER BUG: https://connect.microsoft.com/VisualStudio/feedback/details/1882295
*/

#if 0
namespace royale
{
    template <bool, typename> struct enable_if { };
    template <typename T> struct enable_if<true, T>
    {
        typedef T type;
    };

    template <typename, typename> struct is_same
    {
        static const bool value = false;
    };
    template <typename T> struct is_same<T, T>
    {
        static const bool value = true;
    };

    template <typename T, typename T1, typename ... R>
    struct is_same_or_convertible
    {
        static const bool value =
            is_same<T, T1>::value ||
            std::is_convertible<T, T1>::value ||
            is_same_or_convertible<T, R...>::value;
    };

    template <typename T, typename T1>
    struct is_same_or_convertible<T, T1>
    {
        static const bool value =
            is_same<T, T1>::value ||
            std::is_convertible<T, T1>::value;
    };

    /**
    * Iterator types (Category used during creation)
    * NOT used yet; implemented to distinguish royale iterators
    * from STL iterator
    *
    * The iterators in the container implementations follow the
    * STL and are created by using std::random_access_iterator_tag
    * making them compatible with other STL functions.
    */
    struct input_iterator_tag {};             // Input Iterator
    struct output_iterator_tag {};            // Output Iterator
    struct forward_iterator_tag {};           // Forward Iterator
    struct bidirection_iterator_tag {};       // Bidirectional Iterator
    struct random_access_iterator_tag {};     // Random - access Iterator

    /**
    * Iterator skeleton
    *
    * Holding the basic functions which are shared between all types
    * of royale iterators. Handling virtual functions for incrementing
    * and decrementing pointers.
    * This is the very basic class for handling iterators; based on the STL
    * template definitions which makes it STL compatible - all royale iterators are
    * based in this skeleton and every upcoming iterators will also be based on this
    * skeleton.
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance,              // iterator::difference_type
    class Pointer,               // iterator::pointer
    class Reference>             // iterator::reference
    class royale_iterator_skeleton
    {
    public:
        using value_type = T;
        using difference_type = Distance;
        using pointer = Pointer;
        using reference = Reference;
        using iterator_category = Category;

        DllExport royale_iterator_skeleton(pointer ptr)
            : m_ptr(ptr)
        { }
        DllExport royale_iterator_skeleton(const royale_iterator_skeleton &base)
            : m_ptr(base.m_ptr)
        { }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_iterator_skeleton&
        >::type operator+= (const difference_type rhs)
        {
            m_ptr = increment(rhs);
            return *this;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_iterator_skeleton&
        >::type operator-= (const difference_type rhs)
        {
            m_ptr = decrement(rhs);
            return *this;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            difference_type
        >::type operator+ (const royale_iterator_skeleton &rhs) const
        {
            return (m_ptr + rhs.m_ptr);
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            difference_type
        >::type operator- (const royale_iterator_skeleton &rhs) const
        {
            return (m_ptr - rhs.m_ptr);
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::input_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::forward_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            bool
        >::type operator== (const royale_iterator_skeleton<CurrIteratorCategory, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr == rhs.m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::input_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::forward_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            bool
        >::type operator!= (const royale_iterator_skeleton<CurrIteratorCategory, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr != rhs.m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            bool
        >::type operator< (const royale_iterator_skeleton<CurrIteratorCategory, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr < rhs.m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            bool
        >::type operator>(const royale_iterator_skeleton<CurrIteratorCategory, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr > rhs.m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            bool
        >::type operator<= (const royale_iterator_skeleton<CurrIteratorCategory, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr <= rhs.m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            bool
        >::type operator>= (const royale_iterator_skeleton<CurrIteratorCategory, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr >= rhs.m_ptr;
        }

        /**
        * Returns the current item and increments the internal
        * pointer by n steps - default 1.
        */
        DllExport value_type nextItem(const difference_type stepItems = 1)
        {
            Pointer tmp(m_ptr);
            m_ptr = increment(stepItems);
            return *tmp;
        }

        /**
        * Returns the current item and decrements the internal
        * pointer by n steps - default 1.
        */
        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            value_type
        >::type prevItem(const difference_type stepItems = 1)
        {
            Pointer tmp(m_ptr);
            m_ptr = decrement(stepItems);
            return *tmp;
        }

        /**
        * Increment function - virtual!
        *
        * Used to increment the internal pointer by X steps; this function
        * might be overridden by other iterators (i.e. reverse iterator)
        * \param steps If nothing else was specified the returned pointer
        *        is incremented by "1"; however the function allows stepping
        *        more items within the same execution.
        * \return pointer Returns the pointer to the calculated position
        */
        DllExport virtual Pointer increment(const Distance steps = 1) const
        {
            return (m_ptr + steps);
        }

        /**
        * Decrement function - virtual!
        *
        * Used to increment the internal pointer by X steps; this function
        * might be overridden by other iterators (i.e. reverse iterator)
        * \param steps If nothing else was specified the returned pointer
        *        is incremented by "1"; however the function allows stepping
        *        more items within the same execution.
        * \return pointer Returns the pointer to the calculated position
        */
        DllExport virtual Pointer decrement(const Distance steps = 1) const
        {
            return (m_ptr - steps);
        }

    protected:
        Pointer m_ptr;
    };

    /**
    * Royale base iterator class (basis for all non-const iterators)
    *
    * Holding the basic functions for a non-const iterator.
    * Other iterators can be based on this (if they follow the non-const
    * approach), while this class itself is directly based on the skeleton.
    *
    * The royale non-const forward iterator and the
    * royale non-const reverse iterator are based on this class.
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance,              // iterator::difference_type
    class Pointer,               // iterator::pointer
    class Reference>             // iterator::reference
    class royale_base_iterator : public royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>
    {
    public:
        DllExport royale_base_iterator(Pointer ptr = nullptr) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>(ptr)
        { }

        DllExport royale_base_iterator(const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>(base)
        { }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::input_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::forward_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            T*
        >::type operator->()
        {
            return this->m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::input_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::output_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::forward_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            T&
        >::type operator*()
        {
            return *(this->m_ptr);
        }

        DllExport royale_base_iterator& operator++()
        {
            this->m_ptr = this->increment();
            return *this;
        }

        DllExport royale_base_iterator operator++ (int)
        {
            royale_base_iterator i(*this);
            this->m_ptr = this->increment();
            return i;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_base_iterator&
        >::type operator--()
        {
            this->m_ptr = this->decrement();
            return *this;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_base_iterator
        >::type operator-- (int)
        {
            royale_base_iterator i(*this);
            this->m_ptr = this->decrement();
            return i;
        }


        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            T&
        >::type operator[](const Distance n)
        {
            Pointer tmp = this->increment(n);
            return *tmp;
        }

        DllExport royale_base_iterator<Category, T, Distance, Pointer, Reference> &operator= (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &rhs)
        {
            this->m_ptr = rhs.m_ptr;
            return *this;
        }
    };

    /**
    * Royale base iterator class (basis for all const iterators)
    *
    * Holding the basic functions for a const iterator.
    * Other iterators can be based on this (if they follow the const
    * approach), while this class itself is directly based on the skeleton.
    *
    * The royale const forward iterator and the
    * royale const reverse iterator are based on this class.
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance,              // iterator::difference_type
    class Pointer,               // iterator::pointer
    class Reference>             // iterator::reference
    class royale_base_const_iterator : public royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>
    {
    public:
        DllExport royale_base_const_iterator(Pointer ptr = nullptr) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>(ptr)
        { }

        DllExport royale_base_const_iterator(const royale_base_const_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>(base)
        { }

        DllExport royale_base_const_iterator(const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>(base)
        { }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::input_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::forward_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            const T*
        >::type operator->() const
        {
            return this->m_ptr;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::input_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::output_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::forward_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            const T&
        >::type operator*() const
        {
            return *(this->m_ptr);
        }

        DllExport royale_base_const_iterator& operator++()
        {
            this->m_ptr = this->increment();
            return *this;
        }

        DllExport royale_base_const_iterator operator++ (int)
        {
            royale_base_const_iterator i(*this);
            this->m_ptr = this->increment();
            return i;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_base_const_iterator&
        >::type operator--()
        {
            this->m_ptr = this->decrement();
            return *this;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_base_const_iterator
        >::type operator-- (int)
        {
            royale_base_const_iterator i(*this);
            this->m_ptr = this->decrement();
            return i;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            const T&
        >::type operator[] (const Distance n)
        {
            Pointer tmp = this->increment(n);
            return *tmp;
        }

        DllExport royale_base_const_iterator<Category, T, Distance, Pointer, Reference> &operator= (const royale_base_const_iterator<Category, T, Distance, Pointer, Reference> &rhs)
        {
            this->m_ptr = rhs.m_ptr;
            return *this;
        }
    };

    /**
    * Royale const forward iterator class (used by royale containers)
    *
    * Holding the specific functions for the const forward iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The constant forward iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance = ptrdiff_t,  // iterator::difference_type
    class Pointer = T *,         // iterator::pointer
    class Reference = T &>       // iterator::reference
    class royale_const_iterator : public royale_base_const_iterator <Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_const_iterator<Category, T, Distance, Pointer, Reference>::operator=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_const_iterator(Pointer ptr = nullptr) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference>(ptr)
        { }

        DllExport royale_const_iterator(const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference>(base)
        { }

        DllExport royale_const_iterator next(const Distance stepItems = 1)
        {
            royale_const_iterator tmp(*this);
            this->m_ptr = this->increment(stepItems);
            return tmp;
        }


        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_const_iterator
        >::type prev(const Distance stepItems = 1)
        {
            royale_const_iterator tmp(*this);
            this->m_ptr = this->decrement(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_const_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator + (const Distance value)
        {
            return royale_const_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(this->increment(value));
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_const_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator - (const Distance value)
        {
            return royale_const_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(this->decrement(value));
        }
    };

    /**
    * Royale const reverse iterator class (used by royale containers)
    *
    * Holding the specific functions for the const reverse iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The constant reverse iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance = ptrdiff_t,  // iterator::difference_type
    class Pointer = T *,         // iterator::pointer
    class Reference = T &>       // iterator::reference
    class royale_const_reverse_iterator : public royale_base_const_iterator<Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_const_iterator<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_const_reverse_iterator(Pointer ptr = nullptr) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference>(ptr)
        { }

        DllExport royale_const_reverse_iterator(const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference>(base)
        { }

        DllExport royale_const_reverse_iterator next(const Distance stepItems = 1)
        {
            royale_const_reverse_iterator tmp(*this);
            this->m_ptr = increment(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_const_reverse_iterator
        >::type prev(const Distance stepItems = 1)
        {
            royale_const_reverse_iterator tmp(*this);
            this->m_ptr = decrement(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_const_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator + (const Distance value)
        {
            return royale_const_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(increment(value));
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_const_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator - (const Distance value)
        {
            return royale_const_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(decrement(value));
        }

        DllExport virtual Pointer increment(const Distance steps = 1) const override
        {
            return (this->m_ptr - steps);
        }

        DllExport virtual Pointer decrement(const Distance steps = 1) const override
        {
            return (this->m_ptr + steps);
        }
    };

    /**
    * Royale non-const forward iterator class (used by royale containers)
    *
    * Holding the specific functions for the non-const forward iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The non-const forward iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance = ptrdiff_t,  // iterator::difference_type
    class Pointer = T *,         // iterator::pointer
    class Reference = T &>       // iterator::reference
    class royale_iterator : public royale_base_iterator<Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_iterator<Category, T, Distance, Pointer, Reference>::operator=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_iterator(Pointer ptr = nullptr) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference>(ptr)
        { }

        DllExport royale_iterator(const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference>(base)
        { }

        DllExport royale_iterator next(const Distance stepItems = 1)
        {
            royale_iterator tmp(*this);
            this->m_ptr = this->increment(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_iterator
        >::type prev(const Distance stepItems = 1)
        {
            royale_iterator tmp(*this);
            this->m_ptr = this->decrement(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator + (const Distance value)
        {
            return royale_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(this->increment(value));
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator - (const Distance value)
        {
            return royale_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(this->decrement(value));
        }
    };

    /**
    * Royale non-const reverse iterator class (used by royale containers)
    *
    * Holding the specific functions for the non-const reverse iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The non-const reverse iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
    class T,                     // iterator::value_type
    class Distance = ptrdiff_t,  // iterator::difference_type
    class Pointer = T *,         // iterator::pointer
    class Reference = T &>       // iterator::reference
    class royale_reverse_iterator : public royale_base_iterator<Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_iterator<Category, T, Distance, Pointer, Reference>::operator=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_reverse_iterator(Pointer ptr = nullptr) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference>(ptr)
        { }

        DllExport royale_reverse_iterator(const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference>(base)
        { }

        DllExport royale_reverse_iterator next(const Distance stepItems = 1)
        {
            royale_reverse_iterator tmp(*this);
            this->m_ptr = increment(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport typename std::enable_if<
            (
            std::is_same<CurrIteratorCategory, std::bidirectional_iterator_tag>::value ||
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value
            ),
            royale_reverse_iterator
        >::type prev(const Distance stepItems = 1)
        {
            royale_reverse_iterator tmp(*this);
            this->m_ptr = decrement(stepItems);
            return tmp;
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator + (const Distance value)
        {
            return royale_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(increment(value));
        }

        template<typename... Dummy, typename CurrIteratorCategory = Category>
        DllExport inline typename std::enable_if<
            std::is_same<CurrIteratorCategory, std::random_access_iterator_tag>::value,
            royale_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>
        >::type operator - (const Distance value)
        {
            return royale_reverse_iterator<CurrIteratorCategory, T, Distance, Pointer, Reference>(decrement(value));
        }

        DllExport virtual Pointer increment(const Distance steps = 1) const override
        {
            return (this->m_ptr - steps);
        }

        DllExport virtual Pointer decrement(const Distance steps = 1) const override
        {
            return (this->m_ptr + steps);
        }
    };
}
#endif

namespace royale
{
    /**
    * Iterator skeleton
    *
    * Holding the basic functions which are shared between all types
    * of royale iterators. Handling virtual functions for incrementing
    * and decrementing pointers.
    * This is the very basic class for handling iterators; based on the STL
    * template definitions which makes it STL compatible - all royale iterators are
    * based in this skeleton and every upcoming iterators will also be based on this
    * skeleton.
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance,              // iterator::difference_type
              class Pointer,               // iterator::pointer
              class Reference>             // iterator::reference
    class royale_iterator_skeleton
    {
    public:
        using value_type        = T;
        using difference_type   = Distance;
        using pointer           = Pointer;
        using reference         = Reference;
        using iterator_category = Category;

        DllExport royale_iterator_skeleton (pointer ptr)
            : m_ptr (ptr)
        { }
        DllExport royale_iterator_skeleton (const royale_iterator_skeleton &base)
            : m_ptr (base.m_ptr)
        { }

        DllExport royale_iterator_skeleton &operator+= (const difference_type rhs)
        {
            m_ptr = increment (rhs);
            return *this;
        }

        DllExport royale_iterator_skeleton &operator-= (const difference_type rhs)
        {
            m_ptr = decrement (rhs);
            return *this;
        }

        DllExport difference_type operator+ (const royale_iterator_skeleton &rhs) const
        {
            return (m_ptr + rhs.m_ptr);
        }

        DllExport difference_type operator- (const royale_iterator_skeleton &rhs) const
        {
            return (m_ptr - rhs.m_ptr);
        }

        DllExport bool operator== (const royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr == rhs.m_ptr;
        }

        DllExport bool operator!= (const royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr != rhs.m_ptr;
        }

        DllExport bool operator< (const royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr < rhs.m_ptr;
        }

        DllExport bool operator> (const royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr > rhs.m_ptr;
        }

        DllExport bool operator<= (const royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr <= rhs.m_ptr;
        }

        DllExport bool operator>= (const royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> &rhs) const
        {
            return m_ptr >= rhs.m_ptr;
        }

        /**
        * Returns the current item and increments the internal
        * pointer by n steps - default 1.
        */
        DllExport value_type nextItem (const difference_type stepItems = 1)
        {
            pointer tmp (m_ptr);
            m_ptr = increment (stepItems);
            return *tmp;
        }

        /**
        * Returns the current item and decrements the internal
        * pointer by n steps - default 1.
        */
        DllExport value_type prevItem (const difference_type stepItems = 1)
        {
            pointer tmp (m_ptr);
            m_ptr = decrement (stepItems);
            return *tmp;
        }

        /**
        * Increment function - virtual!
        *
        * Used to increment the internal pointer by X steps; this function
        * might be overridden by other iterators (i.e. reverse iterator)
        * \param steps If nothing else was specified the returned pointer
        *        is incremented by "1"; however the function allows stepping
        *        more items within the same execution.
        * \return pointer Returns the pointer to the calculated position
        */
        DllExport virtual Pointer increment (const Distance steps = 1) const
        {
            return (m_ptr + steps);
        }

        /**
        * Decrement function - virtual!
        *
        * Used to increment the internal pointer by X steps; this function
        * might be overridden by other iterators (i.e. reverse iterator)
        * \param steps If nothing else was specified the returned pointer
        *        is incremented by "1"; however the function allows stepping
        *        more items within the same execution.
        * \return pointer Returns the pointer to the calculated position
        */
        DllExport virtual Pointer decrement (const Distance steps = 1) const
        {
            return (m_ptr - steps);
        }

    protected:
        Pointer m_ptr;
    };

    /**
    * Royale base iterator class (basis for all non-const iterators)
    *
    * Holding the basic functions for a non-const iterator.
    * Other iterators can be based on this (if they follow the non-const
    * approach), while this class itself is directly based on the skeleton.
    *
    * The royale non-const forward iterator and the
    * royale non-const reverse iterator are based on this class.
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance,              // iterator::difference_type
              class Pointer,               // iterator::pointer
              class Reference>             // iterator::reference
    class royale_base_iterator : public royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>
    {
    public:
        DllExport royale_base_iterator (Pointer ptr = nullptr) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> (ptr)
        { }

        DllExport royale_base_iterator (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport T *operator->()
        {
            return this->m_ptr;
        }

        DllExport T &operator*()
        {
            return * (this->m_ptr);
        }

        DllExport royale_base_iterator &operator++()
        {
            this->m_ptr = this->increment();
            return *this;
        }

        DllExport royale_base_iterator operator++ (int)
        {
            royale_base_iterator i (*this);
            this->m_ptr = this->increment();
            return i;
        }

        DllExport royale_base_iterator &operator--()
        {
            this->m_ptr = this->decrement();
            return *this;
        }

        DllExport royale_base_iterator operator-- (int)
        {
            royale_base_iterator i (*this);
            this->m_ptr = this->decrement();
            return i;
        }

        DllExport T &operator[] (const Distance n)
        {
            Pointer tmp = this->increment (n);
            return *tmp;
        }

        DllExport royale_base_iterator<Category, T, Distance, Pointer, Reference> &operator= (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &rhs)
        {
            this->m_ptr = rhs.m_ptr;
            return *this;
        }
    };

    /**
    * Royale base iterator class (basis for all const iterators)
    *
    * Holding the basic functions for a const iterator.
    * Other iterators can be based on this (if they follow the const
    * approach), while this class itself is directly based on the skeleton.
    *
    * The royale const forward iterator and the
    * royale const reverse iterator are based on this class.
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance,              // iterator::difference_type
              class Pointer,               // iterator::pointer
              class Reference>             // iterator::reference
    class royale_base_const_iterator : public royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>
    {
    public:
        DllExport royale_base_const_iterator (Pointer ptr = nullptr) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> (ptr)
        { }

        DllExport royale_base_const_iterator (const royale_base_const_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport royale_base_const_iterator (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_iterator_skeleton<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport const T *operator->() const
        {
            return this->m_ptr;
        }

        DllExport const T &operator*() const
        {
            return * (this->m_ptr);
        }

        DllExport royale_base_const_iterator &operator++()
        {
            this->m_ptr = this->increment();
            return *this;
        }

        DllExport royale_base_const_iterator operator++ (int)
        {
            royale_base_const_iterator i (*this);
            this->m_ptr = this->increment();
            return i;
        }

        DllExport royale_base_const_iterator &operator--()
        {
            this->m_ptr = this->decrement();
            return *this;
        }

        DllExport royale_base_const_iterator operator-- (int)
        {
            royale_base_const_iterator i (*this);
            this->m_ptr = this->decrement();
            return i;
        }

        DllExport const T &operator[] (const Distance n)
        {
            Pointer tmp = this->increment (n);
            return *tmp;
        }

        DllExport royale_base_const_iterator<Category, T, Distance, Pointer, Reference> &operator= (const royale_base_const_iterator<Category, T, Distance, Pointer, Reference> &rhs)
        {
            this->m_ptr = rhs.m_ptr;
            return *this;
        }
    };

    /**
    * Royale const forward iterator class (used by royale containers)
    *
    * Holding the specific functions for the const forward iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The constant forward iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance = ptrdiff_t,  // iterator::difference_type
              class Pointer = T *,         // iterator::pointer
              class Reference = T &>       // iterator::reference
    class royale_const_iterator : public royale_base_const_iterator <Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_const_iterator<Category, T, Distance, Pointer, Reference>::operator=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_const_iterator (Pointer ptr = nullptr) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference> (ptr)
        { }

        DllExport royale_const_iterator (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport royale_const_iterator next (const Distance stepItems = 1)
        {
            royale_const_iterator tmp (*this);
            this->m_ptr = this->increment (stepItems);
            return tmp;
        }

        DllExport royale_const_iterator prev (const Distance stepItems = 1)
        {
            royale_const_iterator tmp (*this);
            this->m_ptr = this->decrement (stepItems);
            return tmp;
        }

        DllExport inline royale_const_iterator<Category, T, Distance, Pointer, Reference> operator + (const Distance value)
        {
            return royale_const_iterator<Category, T, Distance, Pointer, Reference> (this->increment (value));
        }

        DllExport inline royale_const_iterator<Category, T, Distance, Pointer, Reference> operator - (const Distance value)
        {
            return royale_const_iterator<Category, T, Distance, Pointer, Reference> (this->decrement (value));
        }
    };

    /**
    * Royale const reverse iterator class (used by royale containers)
    *
    * Holding the specific functions for the const reverse iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The constant reverse iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance = ptrdiff_t,  // iterator::difference_type
              class Pointer = T *,         // iterator::pointer
              class Reference = T &>       // iterator::reference
    class royale_const_reverse_iterator : public royale_base_const_iterator<Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_const_iterator<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_const_reverse_iterator (Pointer ptr = nullptr) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference> (ptr)
        { }

        DllExport royale_const_reverse_iterator (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_const_iterator<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport royale_const_reverse_iterator next (const Distance stepItems = 1)
        {
            royale_const_reverse_iterator tmp (*this);
            this->m_ptr = increment (stepItems);
            return tmp;
        }

        DllExport royale_const_reverse_iterator prev (const Distance stepItems = 1)
        {
            royale_const_reverse_iterator tmp (*this);
            this->m_ptr = decrement (stepItems);
            return tmp;
        }

        DllExport inline royale_const_reverse_iterator<Category, T, Distance, Pointer, Reference> operator + (const Distance value)
        {
            return royale_const_reverse_iterator<Category, T, Distance, Pointer, Reference> (increment (value));
        }

        DllExport inline royale_const_reverse_iterator<Category, T, Distance, Pointer, Reference> operator - (const Distance value)
        {
            return royale_const_reverse_iterator<Category, T, Distance, Pointer, Reference> (decrement (value));
        }

        DllExport virtual Pointer increment (const Distance steps = 1) const override
        {
            return (this->m_ptr - steps);
        }

        DllExport virtual Pointer decrement (const Distance steps = 1) const override
        {
            return (this->m_ptr + steps);
        }
    };

    /**
    * Royale non-const forward iterator class (used by royale containers)
    *
    * Holding the specific functions for the non-const forward iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The non-const forward iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance = ptrdiff_t,  // iterator::difference_type
              class Pointer = T *,         // iterator::pointer
              class Reference = T &>       // iterator::reference
    class royale_iterator : public royale_base_iterator<Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_iterator<Category, T, Distance, Pointer, Reference>::operator=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_iterator (Pointer ptr = nullptr) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference> (ptr)
        { }

        DllExport royale_iterator (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport royale_iterator next (const Distance stepItems = 1)
        {
            royale_iterator tmp (*this);
            this->m_ptr = this->increment (stepItems);
            return tmp;
        }

        DllExport royale_iterator prev (const Distance stepItems = 1)
        {
            royale_iterator tmp (*this);
            this->m_ptr = this->decrement (stepItems);
            return tmp;
        }

        DllExport inline royale_iterator<Category, T, Distance, Pointer, Reference> operator + (const Distance value)
        {
            return royale_iterator<Category, T, Distance, Pointer, Reference> (this->increment (value));
        }

        DllExport inline royale_iterator<Category, T, Distance, Pointer, Reference> operator - (const Distance value)
        {
            return royale_iterator<Category, T, Distance, Pointer, Reference> (this->decrement (value));
        }
    };

    /**
    * Royale non-const reverse iterator class (used by royale containers)
    *
    * Holding the specific functions for the non-const reverse iterator.
    * Other specific iterators shall not be based on this final iterator
    * anymore.
    *
    * The non-const reverse iterator is used by the royale container classes
    * and is compatible with the rest of the STL implementation concerning iterators
    */
    template <class Category,    // iterator::iterator_category
              class T,                     // iterator::value_type
              class Distance = ptrdiff_t,  // iterator::difference_type
              class Pointer = T *,         // iterator::pointer
              class Reference = T &>       // iterator::reference
    class royale_reverse_iterator : public royale_base_iterator<Category, T, Distance, Pointer, Reference>
    {
    public:
        using royale_base_iterator<Category, T, Distance, Pointer, Reference>::operator=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator+;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator-;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator>=;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<;
        using royale_iterator_skeleton<Category, T, Distance, Pointer, Reference>::operator<=;

        DllExport royale_reverse_iterator (Pointer ptr = nullptr) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference> (ptr)
        { }

        DllExport royale_reverse_iterator (const royale_base_iterator<Category, T, Distance, Pointer, Reference> &base) :
            royale_base_iterator<Category, T, Distance, Pointer, Reference> (base)
        { }

        DllExport royale_reverse_iterator next (const Distance stepItems = 1)
        {
            royale_reverse_iterator tmp (*this);
            this->m_ptr = increment (stepItems);
            return tmp;
        }

        DllExport royale_reverse_iterator prev (const Distance stepItems = 1)
        {
            royale_reverse_iterator tmp (*this);
            this->m_ptr = decrement (stepItems);
            return tmp;
        }

        DllExport inline royale_reverse_iterator<Category, T, Distance, Pointer, Reference> operator + (const Distance value)
        {
            return royale_reverse_iterator<Category, T, Distance, Pointer, Reference> (increment (value));
        }

        DllExport inline royale_reverse_iterator<Category, T, Distance, Pointer, Reference> operator - (const Distance value)
        {
            return royale_reverse_iterator<Category, T, Distance, Pointer, Reference> (decrement (value));
        }

        DllExport virtual Pointer increment (const Distance steps = 1) const override
        {
            return (this->m_ptr - steps);
        }

        DllExport virtual Pointer decrement (const Distance steps = 1) const override
        {
            return (this->m_ptr + steps);
        }
    };
}

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
#include <cstdint>

namespace royale
{
    enum class VariantType
    {
        Int,
        Float,
        Bool
    };

    /*! Implements a variant type which can take different basic data types, the default
     *  type is int and the value is set to zero
     */
    class Variant
    {
    public:
        DllExport Variant();
        DllExport Variant (int n);
        DllExport Variant (float n);
        DllExport Variant (bool n);
        DllExport Variant (VariantType type, uint32_t value);

        DllExport ~Variant();

        DllExport void setFloat (float n);
        DllExport float getFloat() const;

        DllExport void setInt (int n);
        DllExport int getInt() const;

        DllExport void setBool (bool n);
        DllExport bool getBool() const;

        DllExport void setData (VariantType type, uint32_t value);
        DllExport uint32_t getData() const;

        DllExport VariantType variantType() const;

        DllExport bool operator== (const Variant &v) const;
        DllExport bool operator!= (const Variant &v) const;
        DllExport bool operator< (const Variant &v) const;

        /*!
         *  This will be thrown if a wrong type is used
         */
        struct InvalidType {};

    private:
        /*!
         *  Discriminant information about the stored type
         */
        VariantType m_type;

        /*!
         *  Representation of the stored type
         */
        union
        {
            int   i;
            float f;
            bool  b;
        };
    };
}
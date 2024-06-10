---
title: uipc
description: 
generator: doxide
---


# uipc



:material-package: [builtin](builtin/index.md)
:   

:material-package: [geometry](geometry/index.md)
:   

## Types

| Name | Description |
| ---- | ----------- |
| [P](P/index.md) | A different version of weak pointer that allows for easier access to the underlying object  |

## Operators

| Name | Description |
| ---- | ----------- |
| [operator""_GPa](#operator_u0022_u0022_GPa) | Pascal literal operator (GPa)  |
| [operator""_MPa](#operator_u0022_u0022_MPa) | Pascal literal operator (MPa)  |
| [operator""_N](#operator_u0022_u0022_N) | Newton literal operator |
| [operator""_Pa](#operator_u0022_u0022_Pa) | Pascal literal operator |
| [operator""_kPa](#operator_u0022_u0022_kPa) | Pascal literal operator (kPa)  |
| [operator""_km](#operator_u0022_u0022_km) | Meter literal operator (km)  |
| [operator""_m](#operator_u0022_u0022_m) | Meter literal operator |
| [operator""_mm](#operator_u0022_u0022_mm) | Meter literal operator (mm)  |

## Functions

| Name | Description |
| ---- | ----------- |
| [run_length_encode](#run_length_encode) | Run-length encode the input range, the input range must be sorted |

## Operator Details

### operator""_GPa<a name="operator_u0022_u0022_GPa"></a>

!!! function "constexpr long double operator&quot;&quot;_GPa(long double value)"

    
    
    Pascal literal operator (GPa)
     
    
    
    

### operator""_MPa<a name="operator_u0022_u0022_MPa"></a>

!!! function "constexpr long double operator&quot;&quot;_MPa(long double value)"

    
    
    Pascal literal operator (MPa)
     
    
    
    

### operator""_N<a name="operator_u0022_u0022_N"></a>

!!! function "constexpr long double operator&quot;&quot;_N(long double value)"

    
    
    Newton literal operator
    
    ```cpp
    auto force = 1.0_N;
    ```
    
    

### operator""_Pa<a name="operator_u0022_u0022_Pa"></a>

!!! function "constexpr long double operator&quot;&quot;_Pa(long double value)"

    
    
    Pascal literal operator
    
    ```cpp
    auto pressure = 1.0_Pa;
    ```
    
    

### operator""_kPa<a name="operator_u0022_u0022_kPa"></a>

!!! function "constexpr long double operator&quot;&quot;_kPa(long double value)"

    
    
    Pascal literal operator (kPa)
     
    
    
    

### operator""_km<a name="operator_u0022_u0022_km"></a>

!!! function "constexpr long double operator&quot;&quot;_km(long double value)"

    
    
    Meter literal operator (km)
     
    
    
    

### operator""_m<a name="operator_u0022_u0022_m"></a>

!!! function "constexpr long double operator&quot;&quot;_m(long double value)"

    
    
    Meter literal operator
    
    ```cpp
    auto length = 1.0_m;
    ```
    
    

### operator""_mm<a name="operator_u0022_u0022_mm"></a>

!!! function "constexpr long double operator&quot;&quot;_mm(long double value)"

    
    
    Meter literal operator (mm)
     
    
    
    

## Function Details

### run_length_encode<a name="run_length_encode"></a>
!!! function "template &lt;typename InputIt, typename OutputIt, typename OutputCountIt, typename Pred = std::equal_to&lt;&gt;&gt; requires requires(InputIt in_first, InputIt in_last, OutputIt out_unique, OutputCountIt out_counts) { // able to assign value from input to output &#42;out_unique = &#42;in_first; // out_counts must be a iterator to integral type std::integral&lt;typename std::iterator_traits&lt;OutputCountIt&gt;::value_type&gt;; // able to assign value to out_counts &#42;out_counts = 0; } std::size_t run_length_encode(InputIt       in_first, InputIt       in_last, OutputIt      out_unique, OutputCountIt out_counts, Pred&amp;&amp;        pred)"

    
    
    Run-length encode the input range, the input range must be sorted
    
    :material-location-enter: **Parameter** `in_first`
    :    Input iterator to the beginning of the input range
    
    :material-location-enter: **Parameter** `in_last`
    :    Input iterator to the end of the input range
    
    :material-location-enter: **Parameter** `out_unique`
    :    Output iterator to the beginning of the unique values
    
    :material-location-enter: **Parameter** `out_counts`
    :    Output iterator to the beginning of the counts of the unique values
    
    :material-keyboard-return: **Return**
    :    std::size_t The number of unique values
    
    


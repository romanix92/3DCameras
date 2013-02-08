template <class T>
static bool arraysEqual(const T* first, const T* second, size_t len)
{
    const T* end = first + len;
    for(; first != end; ++first, ++second)
        if (*first != *second)
            return false;
    return true;
}
#pragma once

/**
 * WARNING: Deriving from this class is not enough to make a singleton, you also
 * have to declare the constructor of your class private!
 *
 * \code
 * class Foo : public Singleton<Foo>
 * {
 *     friend class Singleton<Foo>;
 * private:
 *     Foo() {} // Ok, private constructor
 * };
 * \endcode
 *
 * Look here for more info on Singletons:
 * https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
 */
template <typename T>
class Singleton
{
public:
    /**
     * \return A reference to the only instance of the class T.
     */
    inline static T& getInstance()
    {
        static T instance;
        return instance;
    }

protected:
    Singleton() {}

public:
    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;
};

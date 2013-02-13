#include <cpptest.h>
#include <cpptest-suite.h>
#include <cpptest-output.h>

#include "ImageTests.h"

int main()
{
    Test::Suite ts;
    ts.add(std::auto_ptr<Test::Suite>(new ImageTest));

    Test::TextOutput output(Test::TextOutput::Verbose);
    return ts.run(output, false);
}
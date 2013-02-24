#include <cpptest.h>
#include <cpptest-suite.h>
#include <cpptest-output.h>

#include "ImageTests.h"
#include "HomographyTest.h"

int main()
{
    Test::Suite ts;
    ts.add(std::auto_ptr<Test::Suite>(new ImageTest));
    ts.add(std::auto_ptr<Test::Suite>(new HomographyTest));

    Test::TextOutput output(Test::TextOutput::Verbose);
    return ts.run(output, false);
}
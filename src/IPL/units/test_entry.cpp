#include <cpptest.h>
#include <cpptest-suite.h>
#include <cpptest-output.h>
//#include <memory>

#include "ImageTests.h"

int main()
{
    Test::Suite ts;
    ts.add(std::auto_ptr<Test::Suite>(new ImageTest));
    //ts.add(auto_ptr<Test::Suite>(new TestSuite2));
    //ts.add(auto_ptr<Test::Suite>(new TestSuite3));

    Test::TextOutput output(Test::TextOutput::Verbose);
    return ts.run(output, false);
}
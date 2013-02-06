#include <cpptest.h>
#include <cpptest-suite.h>
#include <cpptest-output.h>

int main()
{
	Test::Suite ts;
	//ts.add(auto_ptr<Test::Suite>(new TestSuite1));
	//ts.add(auto_ptr<Test::Suite>(new TestSuite2));
	//ts.add(auto_ptr<Test::Suite>(new TestSuite3));

	Test::TextOutput output(Test::TextOutput::Verbose);
	return ts.run(output);
}
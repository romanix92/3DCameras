#include <stdio.h>
#include <stdlib.h>

#define N 10


void sort(int * array, int len);
void mergeSort(int * array, int l, int r);
void merge(int * arr, int l, int r, int m);

int main()
{
    int arr[] = { 10, 1, 9, 2, 8, 3, 7, 4, 6, 5 };
    sort(arr, N);
    for (int i = 0; i < N; ++i)
        printf("%d\n", arr[i]);
}


void sort(int * array, int len)
{
    mergeSort(array, 0, len-1);
}

void mergeSort(int * array, int l, int r)
{
    const int m = (r + l) / 2;
    if (r <= l)
        return;
    mergeSort(array, l, m);
    mergeSort(array, m + 1, r);
    merge(array, l, r, m);
}

void merge(int * arr, int l, int r, int m)
{
    int i, j, k;
    int * buf = (int*)malloc(sizeof(int) * (r + 1));
    printf("%u\n", buf);
    for (i = m + 1; i > l; i--) buf[i - 1] = arr[i - 1];
    for (j = m; j < r; j++) buf[r + m - j] = arr[j + 1];
    for (k = l; k <= r; k++)
        if (buf[j] < buf[i])
            arr[k] = buf[j--];
        else
            arr[k] = buf[i++];
    free(buf);
}



/*#include <cpptest.h>
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
}*/

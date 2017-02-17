#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <fstream>

#include "save_vector.h"

void load_hist(std::string file_name, std::vector< std::vector< float>>& hist);
int get_N_hist(std::string file_name);

int main(int argc, char *argv[])
{

    int N_hist=get_N_hist(argv[1]);
    std::vector< std::vector< float>> hist(N_hist*N_hist*2,std::vector< float>(4,0.0));
    load_hist(argv[1], hist);
    save_vector(hist, "liste_bin.csv");

    FILE* fid = fopen("bins.bin", "wb");
    int nV = N_hist*N_hist*2, nDim = 1;
    fwrite(&nV, sizeof(int), 1, fid);
    fwrite(&nDim, sizeof(int), 1, fid);
    for (int v = 0; v < nV; v++)
    {
        float xyz[3] = {hist[v][0], hist[v][1], hist[v][2]};
        fwrite(xyz, sizeof(float), 3, fid);
        float bin[1]={hist[v][3]};
        fwrite(bin, sizeof(float), 1, fid);
    }
    fclose(fid);
}

void load_hist(std::string file_name, std::vector< std::vector< float>>& hist)
{
    std::ifstream file (file_name);
    int N_hist = get_N_hist(file_name);
    int p=0;
    float delta=M_PI/N_hist;

    for(int i = 0; i < N_hist ; i++)
    {
        for (int j = 0; j < N_hist *2; j++)
        {
            hist[p][0]=cos(j*delta)*sin(i*delta);
            hist[p][1]=sin(j*delta)*sin(i*delta);
            hist[p][2]=cos(i*delta);
            file>>hist[p][3];
            p++;
        }
    }


     file.close();
}

int get_N_hist(std::string file_name)
{
    std::ifstream file (file_name);
    char c;
    int number_of_lines = 0;
    while (file.get(c))
        if (c == '\n')
            ++number_of_lines;

    return number_of_lines;
}

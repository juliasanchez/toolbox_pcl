void save_vector(std::vector<std::vector<float>>& vec, std::string file_name)
{
    std::ofstream file (file_name);
    for(int k = 0; k < vec.size(); k ++)
    {
        for(int n = 0; n < vec[0].size(); n ++)
        {
            file <<vec[k][n] << " " ;
        }
        file <<"\n" ;
    }
    file.close();
}

/*
 * getFilesnames_script.cpp
 *
 *  Created on: Dec 22, 2013 by Gonzalo Ferrer. 
 *  Last Modified by Ely Repiso on 2025 (migration to ros-Noetic and in the middle of ros2-humble migration)
 *      Author: Initial code of the Robot alone AKP navigation is from Gonzalo Ferrer. 
 *      Author: The Addition of ASPSI and other approaching and accompaniment modifications and codes are of Ely Repiso.
 *
 *      License (for other authors that will not be the original ones): CC BY-NC-ND 4.0 
 *              (Attribution-NonCommercial-NoDerivatives 4.0 International)
 *              https://creativecommons.org/licenses/by-nc-nd/4.0/deed.en
 *
 *      This license does not allow other authors to modify or to take profit from these works. 
 *      Then, for modifications or derivative works, please contact ely.repiso@upc.edu to try to agree on 
 *   collaborations (for journals with other researchers, formal collaborations between UPC and companies, and so on).
 *
 *  Please to only use it cite: Repiso, Ely, Anaís Garrell, and Alberto Sanfeliu. "Adaptive social planner to accompany people in real-life dynamic environments." International Journal of Social Robotics 16.6 (2024): 1189-1221.
 *
 *  Redistribution and use in source and binary forms, without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission. 
 *   * Additionally, remember that the derivatives without collaboration 
 *     Of the original authors are prohibited
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#include<iostream>
#include<string>
#include<vector>
#include <dirent.h>
#include <stdio.h>


std::string addPath(std::string a, std::string c){
    std::string b = "/";
    return a+b+c;
}

std::vector<std::string> folderStream(std::string PATH){
    std::vector<std::string> directories;
    DIR *dir = opendir(PATH.c_str());
    struct dirent *entry = readdir(dir);
    while (entry != NULL)
    {
        if (entry->d_type == DT_DIR){
            std::string aux = entry->d_name;
            if(aux.compare(".") != 0 and aux.compare("..")!=0){
                directories.push_back(entry->d_name);
            }
        }
        entry = readdir(dir);
    }

    closedir(dir);

    return directories;
}

void pathGeneratoraux(std::string path, int level, std::vector<std::string> &paths){
    std::vector<std::string> v = folderStream(path);
    if(level == 0){
        for(int i = 0; i<v.size();i++){
            paths.push_back(addPath(path,v[i]));
        }
    }
    else{
        for(int i = 0; i<v.size();i++){
            pathGeneratoraux(addPath(path,v[i]),level-1,paths);
        }
    }
}

std::vector<std::string> pathGenerator(std::string path){
    std::vector<std::string> paths;
    pathGeneratoraux(path,1,paths);
    return paths;
}

int main(int argc,char *argv[]){
    std::string path = "/home/aserra/Documents/IRI/Database_creation/Experiments";
    std::vector<std::string> paths = pathGenerator(path);
    for(int i = 0; i<paths.size(); i++){
        std::cout<<paths[i]<<std::endl;
    }
}

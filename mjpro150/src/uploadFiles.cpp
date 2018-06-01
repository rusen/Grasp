// Application that uploads anything in current data folder to the web server.
#include <iostream>
#include <time.h>
#include <string>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <fstream>
#include <algorithm>    // std::copy
#include <vector>
#include <thread>
#include <boost/filesystem.hpp>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Infos.hpp>

#define SERVER_ADDRESS "http://52.14.167.90:8000/"

// Function to send files to server (Blocking)
// name should be the absolute path (client side).
bool UploadFile(const char *name);
void DistributePoints(const char * path);
void UploadExtraFiles(const char * path);
void RemoveOldFolders(const char * path);
void RemoveOldPoints(const char * path);

int main(int argc, const char** argv){

	while(true)
	{
		// Check points and distribute them to Grasp server clients.
		DistributePoints(argv[1]);

	    // There may be extra files that need to be uploaded
	    // (by other clients which do not have direct internet access).
	    // Upload them too.
	    UploadExtraFiles(argv[1]);

	    try{
			// Remove old folders from dropbox
			RemoveOldFolders(argv[1]);

			// Remove old folders from dropbox
			RemoveOldPoints(argv[1]);
	    }
	    catch(std::exception & e){

	    }

	    // Sleep for 10 seconds and try again.
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}
}


void DistributePoints(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/points");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	time_t curTime = time(NULL);
	std::vector<std::string> emptyDirs;

	// Create folders for up to 8 servers, if they don't exist
	for (int i = 0; i<8; i++)
	{
		char tmp[1000];
		strcpy(tmp, dropboxBase);
		strcat(tmp, "/server/");
		char str[10];
		sprintf(str, "%d", i);
		strcat(tmp, str);

		bool dirEmpty = true;
		// If directory doesn't exist, create. If it does, check if it has any pcd files in it.
		if (!boost::filesystem::is_directory(tmp))
				boost::filesystem::create_directories(tmp);
		else{
			// If this directory is empty (no point files), mark it empty.
			boost::filesystem::path p3(tmp);

			// Check if this folder has any pcd files in it
			for (auto k = boost::filesystem::directory_iterator(p3); k != boost::filesystem::directory_iterator(); k++)
			{
				if ((k->path().string().find(".pcd") !=std::string::npos))
				{
					// Mark directory full
					dirEmpty = false;
					break;
				}
			}
		}

		// If no files, mark it empty
		if (dirEmpty)
			emptyDirs.push_back(std::string(tmp));
	}

	// Shuffle emptyDirs
	if (emptyDirs.size() > 0)
		std::random_shuffle ( emptyDirs.begin(), emptyDirs.end() );

	// Get file list and ages in seconds
	boost::filesystem::path p(tmpStr);
	char tmp[1000];
	std::vector<time_t> ages;
	std::vector<std::string> filePaths;

	// Check if this folder has any pcd files in it
	for (auto k = boost::filesystem::directory_iterator(p); k != boost::filesystem::directory_iterator(); k++)
	{
		if ((k->path().string().find(".pcd") !=std::string::npos))
		{
			time_t fileTime = boost::filesystem::last_write_time(k->path());
			ages.push_back(curTime - fileTime);
			filePaths.push_back(k->path().string());
		}
	}

	// Assign point files into empty server folders
	while(true)
	{
		// No empty dirs: break
		if (!emptyDirs.size() || !filePaths.size())
			break;

		// Find oldest file
		std::string oldestFile = filePaths[0];
		time_t age = ages[0];
		int index = 0;

		// Check if this folder has any pcd files in it
		for (int k = 1; k<ages.size(); k++)
		{
			if (ages[k] > age)
			{
				index = k;
				age = ages[k];
				oldestFile = filePaths[k];
			}
		}

		std::cout<<"Assigning " << oldestFile << " to server " << emptyDirs[0].c_str() << std::endl;
		// Assign the point file to the first empty directory
		boost::filesystem::path pFile(oldestFile.c_str());
		char newFile[1000];
		strcpy(newFile, emptyDirs[0].c_str());
		strcat(newFile, "/");
		strcat(newFile, pFile.filename().c_str());
		boost::filesystem::copy_file (oldestFile.c_str(), newFile, boost::filesystem::copy_option::overwrite_if_exists);
		boost::filesystem::remove(oldestFile.c_str());

		// Remove entries
		emptyDirs.erase(emptyDirs.begin());
		ages.erase(ages.begin() + index);
		filePaths.erase(filePaths.begin() + index);
	}
}

void UploadExtraFiles(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/upload");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	// Try to upload all files under the $dropboxBase$/upload folder to the system.
	boost::filesystem::path p(tmpStr);
	bool uploadFlag = true;
	char tmp[1000];

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if (i->path().extension().string() == ".zip")
		{
			std::cout<<"Uploading "<<i->path().string().c_str()<<std::endl;
			uploadFlag = uploadFlag && UploadFile(i->path().string().c_str());
			if (uploadFlag)
			{
				boost::filesystem::remove(i->path().string().c_str());
			}
			else
				break;
		}
	}
}


void RemoveOldFolders(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/data");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	// Try to upload all files under the $dropboxBase$/upload folder to the system.
	boost::filesystem::path p(tmpStr);
	bool uploadFlag = true;
	char tmp[1000];
	time_t curTime = time(NULL);

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 1800){
			boost::filesystem::remove_all(i->path());
		}
	}
}

void RemoveOldPoints(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/points");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	// Try to upload all files under the $dropboxBase$/upload folder to the system.
	boost::filesystem::path p(tmpStr);
	bool uploadFlag = true;
	char tmp[1000];
	time_t curTime = time(NULL);

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 360){
			boost::filesystem::remove_all(i->path());
		}
	}
}

bool UploadFile(const char *name){

	if (!boost::filesystem::exists(name))
	{
		std::cout<<"File "<<name<<" does not exist!"<<std::endl;
 		return true;
	}

	bool flag = false;
	try{
		curlpp::Cleanup cleaner;
		curlpp::Easy request;
		char buf[1000];
		buf[0] = 0;

	    // Get file size.
	    std::ifstream sizeStream( name, std::ios::binary | std::ios::ate);
	    int contentLength = sizeStream.tellg();
	    sizeStream.close();

	    // Create header.
		std::list< std::string > headers;
		headers.push_back("Content-Type: application/octet-stream");
		sprintf(buf, "Content-Length: %d", contentLength);
		headers.push_back(buf);

		// Set up request options.
		char remotePath[1000];
		remotePath[0] = 0;
		strcat(remotePath, SERVER_ADDRESS);

		// Get filename and extension.
		char tempName[1000];
		strcpy(tempName, name);
		char * fn = strrchr(tempName, '/') + 1;
		strcat(remotePath, fn);
		std::cout<<"Remote path:"<<remotePath<<std::endl;
	    std::ifstream myStream( name, std::ios::binary);

		request.setOpt(new curlpp::options::Url(remotePath));
		request.setOpt(new curlpp::options::Verbose(true));
		request.setOpt(new curlpp::options::ReadStream(&myStream));
		request.setOpt(new curlpp::options::InfileSize(contentLength));
		request.setOpt(new curlpp::options::Upload(true));

		// Create header.
		request.setOpt(new curlpp::options::HttpHeader(headers));
	    std::cout<<"Set everything: "<<std::endl;

		// Arguments.
		request.perform();
		int code = curlpp::infos::ResponseCode::get(request);
		if (code == 200)
			flag = true;
		else{
			std::cout<<"Connector Upload failed: "<<code<<", filename: "<<name<<std::endl;
			return false;
		}
		std::cout<<code<<std::endl;

		// Close mystream and remove file.
		myStream.close();
	}
    catch ( curlpp::LogicError & e ) {
      std::cout << e.what() << std::endl;
    }
    catch ( curlpp::RuntimeError & e ) {
      std::cout << e.what() << std::endl;
    }
    return flag;
}

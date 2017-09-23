/*
 * Connector.cpp
 *
 *  Created on: 22 Aug 2017
 *      Author: rusi
 */

#include "util/Connector.h"
#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Infos.hpp>
#include <thread>
#include <chrono>
#include <stdio.h>
#include <unistd.h>

namespace Grasp {

Connector::Connector() {
	// TODO Auto-generated constructor stub
}

Connector::~Connector() {
	// TODO Auto-generated destructor stub
}

// Function that uploads a file to the server.
bool Connector::UploadFile(const char *name){

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

		// Arguments.
		request.perform();
		int code = curlpp::infos::ResponseCode::get(request);
		if (code == 200)
			flag = true;
		else{
			std::cout<<"Connector Upload failed: "<<code<<", filename: "<<name<<std::endl;
			return false;
		}

		// Close mystream and remove file.
		myStream.close();
//		remove(name);
	}
    catch ( curlpp::LogicError & e ) {
      std::cout << e.what() << std::endl;
    }
    catch ( curlpp::RuntimeError & e ) {
      std::cout << e.what() << std::endl;
    }
    return flag;
}

bool Connector::UploadFileToDropbox(const char * fileId, const char *name, const char * dropboxFolder){

	if (!boost::filesystem::exists(name))
	{
		std::cout<<"File "<<name<<" does not exist!"<<std::endl;
 		return true;
	}

	char newName[100];
	strcpy(newName, dropboxFolder);
	strcat(newName, "/points/");
	strcat(newName, fileId);
	strcat(newName, ".pcd");
	boost::filesystem::copy_file(name, newName);
	std::cout<<"File successfully uploaded to Dropbox."<<std::endl;
	return true;
}

bool Connector::DownloadFileFromDropbox(const char *name){
	bool flag = access( name, F_OK ) != -1;
	std::this_thread::sleep_for(std::chrono::seconds(1));
	return flag;
}

// Function that downloads a file from the server
bool Connector::DownloadFile(const char *name){

	 // Next flag is set to 1 if the name field is emtpy.
	bool nextFlag = !std::strcmp(name, "");

	// Download file.
	bool flag = false;
	try{
		curlpp::Cleanup cleaner;
		curlpp::Easy request;
		char fileName[50];

		// If we're requesting a known file, no need to generate a temporary name. Otherwise create one.
		if (nextFlag)
		{
			// Obtain file name of local file.
			fileName[0] = 0;
			strcat(fileName, "./tmp/");

			char tmp[] = "XXXXXX";
			mktemp(tmp);
			strcat(fileName, tmp);
		} else{
			strcpy(fileName, name);
		}

	    // Create header.
	    std::ofstream myStream( fileName, std::ios::binary);
		std::list< std::string > headers;
		headers.push_back("Content-Type: application/octet-stream");

		// Identify remote file.
		char remotePath[100];
		remotePath[0] = 0;
		strcat(remotePath, SERVER_ADDRESS);

		if (nextFlag)
			strcat(remotePath, "next");
		else
		{
			char tempName[1000];
			strcpy(tempName, name);
			char * fn = strrchr(tempName, '/') + 1;
			strcat(remotePath, fn);
		}

	    // Add file info to the header.
		request.setOpt(new curlpp::options::Url(remotePath));
		request.setOpt(new curlpp::options::WriteStream(&myStream));

		// Create header.
		request.setOpt(new curlpp::options::HttpHeader(headers));

		// Arguments.
		request.perform();
		int code = curlpp::infos::ResponseCode::get(request);
		if (code == 200)
			flag = true;
		else
		{
			// Delete file, as we couldn't download anything.
			remove(fileName);
			std::cout<<"Connector Download failed: "<<code<<", filename: "<<fileName<<std::endl;
			return false;
		}
		// Close mystream.
		myStream.close();

		if (nextFlag)
		{
			// Get the relevant file name, trim and rename the file.
			std::ifstream istr(fileName, std::ios::binary);
			char newName[7];
			newName[6] = 0;
			istr.read(newName, 6);
			istr.seekg(8);

			// Create new file name and output stream.
			char fileNameNew[50];
			fileNameNew[0] = 0;
			strcat(fileNameNew, "./tmp/");
			strcat(fileNameNew, newName);
			strcat(fileNameNew, ".pcd");

			// Write relevant bytes into new file.
			std::ofstream newOutStream(fileNameNew, std::ios::binary);
			newOutStream << istr.rdbuf();

			// Close streams.
			istr.close();
			newOutStream.close();

			// Delete temp file.
			remove(fileName);
		}
	}
    catch ( curlpp::LogicError & e ) {
      std::cout << e.what() << std::endl;
    }
    catch ( curlpp::RuntimeError & e ) {
      std::cout << e.what() << std::endl;
    }
    return flag;
}

} /* namespace Grasp */

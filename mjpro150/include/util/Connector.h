/*
 * Connector.h
 *
 *  Created on: 22 Aug 2017
 *      Author: rusi
 */

#ifndef UTIL_CONNECTOR_H_
#define UTIL_CONNECTOR_H_

#define SERVER_ADDRESS "http://52.14.167.90:8000/"
#define DROPBOX_FOLDER "/Users/rusi/TestDropbox/Dropbox/"
#define USING_DROPBOX true

namespace Grasp {

class Connector {
public:
	Connector();
	virtual ~Connector();

	// Function to send files to server (Blocking)
	// name should be the absolute path (client side).
	static bool UploadFile(const char *name);

	// Workaround solution to communicate with Sulley.
	static bool UploadFileToDropbox(const char * fileId, const char *name);

	// Function to get files from server. (Unblocking)
	// If you wish to have next set of points, name field should be "".
	// The file name should be an absolute path otherwise (on client side)
	static bool DownloadFile(const char *name);

	// Function to get files from server. (Unblocking)
	// Workaround to commnicate with Sulley.
	static bool DownloadFileFromDropbox(const char *name);

};

} /* namespace Grasp */

#endif /* UTIL_CONNECTOR_H_ */

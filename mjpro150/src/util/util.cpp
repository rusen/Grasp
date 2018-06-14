#include <util/util.h>
#include <util/Connector.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <boost/filesystem.hpp>

namespace Grasp{

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

// Function to create relevant XMLs with random parameters.
std::string CreateXMLs(const char * base, GraspPlanner * planner, int objectId, int baseId, int runCount, bool reSimulateFlag){
	// Templates for model, base and assets files are already there.
	// We just need to modify them by adding the variations.
//	std::cout<<"RANDOM NO BEFORE SRAND"<<rand()<<std::endl;
//	srand(time(NULL));

	// IF this folder exists already, we can assume that the relevant xml files should already exist.
	// We just need to copy them to the relevant folders.
	if (reSimulateFlag)
	{
		std::string outputStr;
		char modelFolder[1000];
		strcpy(modelFolder, planner->baseFolder);
		strcat(modelFolder, "/model");

		boost::filesystem::path p(modelFolder);

		// Go over the files and delete them one by one.
		for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
		{
			if (i->path().string().find("_include_object") !=std::string::npos &&
					!(i->path().string().find("_include_object.xml") !=std::string::npos ||
						i->path().string().find("_include_object_assets") !=std::string::npos ))
			{
				char tmp[1000];
				strcpy(tmp, base);
				strcat(tmp, "/tmp/");
				strcat(tmp, i->path().filename().c_str());
				boost::filesystem::copy_file(i->path(), tmp, boost::filesystem::copy_option::overwrite_if_exists);
			}
			else if (i->path().string().find(planner->fileId) !=std::string::npos)
			{
				char tmp[1000];
				strcpy(tmp, base);
				strcat(tmp, "/");
				strcat(tmp, i->path().filename().c_str());
				boost::filesystem::copy_file(i->path(), tmp, boost::filesystem::copy_option::overwrite_if_exists);

				if (i->path().string().find("Test") != std::string::npos)
					outputStr = std::string(tmp);
			}
		}

		// Return test file path
		return outputStr;
	}

	// Allocate space for model xml.
	char modelXMLStr[10000];
	modelXMLStr[0] = 0;
	sprintf(modelXMLStr, "<mujoco model=\"BHAM\">\n    <include file=\"include_BHAM.xml\"/>\n    <include file=\"include_assets.xml\"/>\n");

    // Manage file names
    char baseIdFile[1000], classIdFile[1000], oldAssetFile[1000], oldBaseAssetFile[1000], newAssetFile[1000],
	newBaseAssetFile[1000], oldObjectFile[1000], oldBaseFile[1000], newObjectFile[1000], newBaseFile[1000],
	tmp[1000], tmpXML[1000], lightFile[1000], tableFile[1000], modelPrefix[1000], tmpModelPrefix[1000];

    // Get base id file name
    strcpy(baseIdFile, base);
    strcat(baseIdFile, "/baseIds.txt");

    // Read base id
    int baseType = 0;
    FILE * fid = fopen(baseIdFile, "r");
    for (int i = 0; i<objectId; i++)
    	fscanf(fid, "%d\n", &baseType);
    fclose(fid);

    // Get base id file name
    strcpy(classIdFile, base);
    strcat(classIdFile, "/classIds.txt");

    // Read class id
    int classId = 0;
    fid = fopen(classIdFile, "r");
    for (int i = 0; i<objectId; i++)
    	fscanf(fid, "%d\n", &classId);
    fclose(fid);

    // Obtain a filename prefix for this specific object
    strcpy(modelPrefix, base);
    strcat(modelPrefix, "/");
    strcat(modelPrefix, planner->fileId);
    strcat(modelPrefix, "_");

    // Obtain a temp filename prefix for this specific object
    strcpy(tmpModelPrefix, base);
    strcat(tmpModelPrefix, "/tmp");

    // Create tmp folder if it doesn't exist
    if (!boost::filesystem::exists(tmpModelPrefix))
    	boost::filesystem::create_directories(tmpModelPrefix);

    // Set prefix
    strcat(tmpModelPrefix, "/");
    strcat(tmpModelPrefix, planner->fileId);
    strcat(tmpModelPrefix, "_include_object");

    // Get light and table for this specific object
    strcpy(tmp, base);
    strcat(tmp, "/include_lightOrg.xml");
    strcpy(lightFile, modelPrefix);
    strcat(lightFile, "include_light.xml");
    sprintf(tmpXML, "    <include file=\"%s_include_light.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(tmp, lightFile, boost::filesystem::copy_option::overwrite_if_exists);
    strcpy(tmp, base);
    strcat(tmp, "/include_tableOrg.xml");
    strcpy(tableFile, modelPrefix);
    strcat(tableFile, "include_table.xml");
    sprintf(tmpXML, "    <include file=\"%s_include_table.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(tmp, tableFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Object model
    strcpy(newObjectFile, modelPrefix);
    strcat(newObjectFile, "include_object.xml");
    strcpy(oldObjectFile, base);
    strcat(oldObjectFile, "/mesh/objects/");
    sprintf(tmp, "D_%d/D_%d_object.xml", objectId, objectId);
    strcat(oldObjectFile, tmp);
    sprintf(tmpXML, "    <include file=\"%s_include_object.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(oldObjectFile, newObjectFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Mass info
    char massFile[1000],tmpStr10[1000];
    strcpy(massFile, base);
    strcat(massFile, "/mesh/objects/");
    sprintf(tmpStr10, "D_%d/D_%d_mass.txt", objectId, objectId);
    strcat(massFile, tmpStr10);

    int numberOfParts;
    float partMasses[5000], totalMass = 0;
    FILE * massFileID = fopen(massFile, "r");
    fscanf(massFileID, "%d\n", &numberOfParts);
    for (int itr = 0; itr<numberOfParts; itr++)
    {
    	fscanf(massFileID, "%f\n", &(partMasses[itr]));
    	totalMass += partMasses[itr];
    }
    fclose(massFileID);

    // Object assets
    strcpy(newAssetFile, modelPrefix);
    strcat(newAssetFile, "include_object_assets.xml");
    strcpy(oldAssetFile, base);
    strcat(oldAssetFile, "/mesh/objects/");
    sprintf(tmp, "D_%d/D_%d_assets.xml", objectId, objectId);
    strcat(oldAssetFile, tmp);
    sprintf(tmpXML, "    <include file=\"%s_include_object_assets.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(oldAssetFile, newAssetFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Depending on the base type, get relevant base files.
    if (baseType == 1)
    {
    	// Set minimum z to just above the base platform (usually a cup)
    	planner->minPointZ = -0.255; // Table is at -0.35, each base is about 9 cms.

    	// Base file
        strcpy(newBaseFile, modelPrefix);
        strcat(newBaseFile, "include_base.xml");
        strcpy(oldBaseFile, base);
        strcat(oldBaseFile, "/mesh/bases/");
        sprintf(tmp, "D_%d/D_%d_base.xml", baseId, baseId);
        strcat(oldBaseFile, tmp);
        sprintf(tmpXML, "    <include file=\"%s_include_base.xml\"/>\n", planner->fileId);
        strcat(modelXMLStr, tmpXML);
        boost::filesystem::copy_file(oldBaseFile, newBaseFile, boost::filesystem::copy_option::overwrite_if_exists);

		// Base assets
		strcpy(newBaseAssetFile, modelPrefix);
		strcat(newBaseAssetFile, "include_base_assets.xml");
		strcpy(oldBaseAssetFile, base);
		strcat(oldBaseAssetFile, "/mesh/bases/");
		sprintf(tmp, "D_%d/D_%d_assets.xml", baseId, baseId);
		strcat(oldBaseAssetFile, tmp);
        sprintf(tmpXML, "    <include file=\"%s_include_base_assets.xml\"/>\n", planner->fileId);
        strcat(modelXMLStr, tmpXML);
		boost::filesystem::copy_file(oldBaseAssetFile, newBaseAssetFile, boost::filesystem::copy_option::overwrite_if_exists);
    }
    else if (baseType == 2)
    {
    	planner->minPointZ = -0.315; // 1 cm above invisible base.
    	strcat(modelXMLStr, "    <include file=\"include_baseInv.xml\"/>\n");
    }

    // Finish and print model xml.
    strcat(modelXMLStr, "</mujoco>");
    strcpy(tmp, modelPrefix);
    strcat(tmp, "Test.xml");
	std::ofstream tOut(tmp);
	tOut<<modelXMLStr;
	tOut.close();

	// Create output string.
	std::string outputStr(tmp);

	// Set friction upper limit.
	float frictionLowerLimit = 0.5;
	float frictionUpperLimit = 1;

	// Create random scale based on the object type.
	float xScale = RF*0.1+0.95;
	float yScale = xScale;
	float zScale = xScale;
	float eulerx = 0, eulery = 0, eulerz = RF * 2 * M_PI;
	char lowerStr[20];
	strcpy(lowerStr, "pos=\"0 0 -0.15\"");

	if (classId == 7 || classId == 10 || classId == 13 || classId == 15 || classId == 16 )
	{
		strcpy(lowerStr, "pos=\"0 0 0\"");
	}
	if (classId == 5) // Cup and Funnel can be of any direction.
	{
		if (rand()%2 == 1)
			eulerx = RF * 2 * M_PI, eulery = RF * 2 * M_PI;
	}
	if (classId == 22)
	{
		eulerx = RF * 2 * M_PI, eulery = RF * 2 * M_PI;
	}

	// Scale the object randomly.
	std::ifstream tAsset(newAssetFile);
	std::string assetStr((std::istreambuf_iterator<char>(tAsset)),
					 std::istreambuf_iterator<char>());
	tAsset.close();

	// Replace scale
	sprintf(tmp, "<mesh scale=\"%f %f %f\" ", xScale, yScale, zScale);
	replaceAll(assetStr, std::string("<mesh "), std::string(tmp));

	// Write light string.
	std::ofstream tAssetOut(newAssetFile);
	tAssetOut<<assetStr;
	tAssetOut.close();

	// For each run, we create a different object with varying friction and weight.
	for (int runItr = 0; runItr<runCount; runItr++)
	{
		float frictionRange = RF, weightRange = RF;
		if (runCount == 4)
		{
			switch(runItr)
			{
				case 0:
					frictionRange = 0;
					weightRange = 1;
					break;
				case 1:
					frictionRange = 1;
					weightRange = 1;
					break;

				case 2:
					frictionRange = 0;
					weightRange = 0;
					break;

				case 3:
					frictionRange = 1;
					weightRange = 0;
					break;
			}
		}

		// Create random parameters
		float friction = frictionRange * (frictionUpperLimit - frictionLowerLimit) + frictionLowerLimit;

		// Get the right object file.
		boost::filesystem::copy_file(oldObjectFile, newObjectFile, boost::filesystem::copy_option::overwrite_if_exists);

		// Modify model file.
		std::ifstream t(newObjectFile);
		std::string objectStr((std::istreambuf_iterator<char>(t)),
						 std::istreambuf_iterator<char>());

		// Replace friction
		sprintf(tmp, "friction=\"%f %f %f\"", friction, friction*0.005, friction*0.0001);
		replaceAll(objectStr, std::string("friction=\"\""), std::string(tmp));

		// Create random mass, depending on the object type.
		float baseWeights[] =  {20,  50,  50, 200,  30, 0, 40, 150,  80,  50, 250, 40,  50, 20, 40, 40, 150, 0, 0, 0, 500, 40, 50, 100};
		float addedWeights[] = {60, 350, 450, 200, 300, 0, 60, 350, 120, 100, 100, 180, 100,  180, 60, 60, 100, 0, 0, 0, 300, 40, 20,  50};
		int baseWeight = baseWeights[classId-1];
		int addedWeight = addedWeights[classId-1];
		float mass = (float)( weightRange*(float)addedWeight + (float)baseWeight);

		std::cout<<"MASS OF THE OBJECT:"<<mass<<std::endl;

		mass = mass/1000;

		// We find and save relative mass of each file.
		float multiply = mass / totalMass;
		for (int itr = 0; itr < numberOfParts; itr++)
		{
			char tmpStr[100], tmpStr2[100], tmpStr3[100];
			sprintf(tmpStr2, "%d", itr+1);
			strcpy(tmpStr, "mass=\"");
			strcat(tmpStr, tmpStr2);
			strcat(tmpStr, "\"");

			sprintf(tmpStr2, "%f", multiply * partMasses[itr]);
			strcpy(tmpStr3, "mass=\"");
			strcat(tmpStr3, tmpStr2);
			strcat(tmpStr3, "\"");
			replaceAll(objectStr, std::string(tmpStr), std::string(tmpStr3));
		}

		// Create random orientation
		sprintf(tmp, "euler=\"%f %f %f\"", eulerx, eulery, eulerz);
		replaceAll(objectStr, std::string("euler=\"\""), std::string(tmp));

		// Lower the object's initial position
		replaceAll(objectStr, std::string("pos=\"0 0 0\""), std::string(lowerStr));

		// Create random object colour
		float red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
		sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
		replaceAll(objectStr, std::string("rgba=\"\""), std::string(tmp));

		// Write object file.
		t.close();
		std::ofstream tOutObject(newObjectFile);
		tOutObject<<objectStr;
		tOutObject.close();

		// Finally, move the file into a sub-folder so that we do not clutter the workspace.
		char tmp[1000];
		strcpy(tmp, tmpModelPrefix);
		strcat(tmp, std::to_string(runItr).c_str());
		strcat(tmp, ".xml");
		boost::filesystem::rename(newObjectFile, tmp);
	}

	// Moving on to the light.
	std::ifstream tLight(lightFile);
	std::string lightStr((std::istreambuf_iterator<char>(tLight)),
	                 std::istreambuf_iterator<char>());
	tLight.close();
	// Assign intensity [ 0.5 - 1]
	float intensity = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	intensity = 0.8;
	sprintf(tmp, "diffuse=\"%f %f %f\"", intensity, intensity, intensity);
	replaceAll(lightStr, std::string("diffuse=\"\""), std::string(tmp));

	// Specular intensity of the light
	intensity = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	intensity = intensity/4;
	sprintf(tmp, "specular=\"%f %f %f\"", intensity, intensity, intensity);
	replaceAll(lightStr, std::string("specular=\"\""), std::string(tmp));

	// Assign location (-0.5:0.5, -0.5:0.5, 1:2)
	float x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	x = x - 0.5;
	float y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	y = y - 0.5;
	float z = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	z = z + 0.9;
	x=0, y=1, z=1;
	sprintf(tmp, "pos=\"%f %f %f\"", x, y, z);
	replaceAll(lightStr, std::string("pos=\"\""), std::string(tmp));

	// Assign lookat point for the light.
	x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	x = (x) - 0.5;
	y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	y = (y) - 0.5;
	z = -1;
	sprintf(tmp, "dir=\"%f %f %f\"", x, y, z);
	replaceAll(lightStr, std::string("dir=\"\""), std::string(tmp));

	// Write light string.
	std::ofstream tLightOut(lightFile);
	tLightOut<<lightStr;
	tLightOut.close();

	// Fix the table colour as well.
	std::ifstream tTable(tableFile);
	std::string tableStr((std::istreambuf_iterator<char>(tTable)),
	                 std::istreambuf_iterator<char>());
	tTable.close();
	intensity = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	intensity = intensity/2 + 0.3; // [0.3 - 0.8]
//	sprintf(tmp, "rgba=\"%f %f %f 1\"", intensity, intensity, intensity);
	sprintf(tmp, "rgba=\"0.93 0.87 0.8 1\"");
	replaceAll(tableStr, std::string("rgba=\"\""), std::string(tmp));

	// Write table string.
	std::ofstream tTableOut(tableFile);
	tTableOut<<tableStr;
	tTableOut.close();

	if (baseType == 1)
	{
		// Finally, we fix the base colour.
		std::ifstream tBase(newBaseFile);
		std::string baseStr((std::istreambuf_iterator<char>(tBase)),
						 std::istreambuf_iterator<char>());
		tBase.close();
		float red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
		sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
		replaceAll(baseStr, std::string("rgba=\"\""), std::string(tmp));

		// Write table string.
		std::ofstream tBaseOut(newBaseFile);
		tBaseOut<<baseStr;
		tBaseOut.close();
	}

	// Return name of the entry file.
	return outputStr;
}

void UploadFiles(const char * base, GraspPlanner * planner, int objectId, int baseId, bool keepCollidingGrasps, bool reSimulateFlag){

	std::cout<<"BASE FOLDER:"<<planner->baseFolder<<std::endl;
	char viewFolder[1000], imageFolder[1000], dataFile[1000], trjFile[1000];
	strcpy(viewFolder, planner->baseFolder);
	strcat(viewFolder, "/views/");
	strcpy(imageFolder, planner->baseFolder);
	strcat(imageFolder, "/images/");
	if (!boost::filesystem::exists(imageFolder))
		boost::filesystem::create_directories(imageFolder);

	// First, we need to prepare the grasp data and images.
	// We will create a bin file that includes object class,
	// object id, grasp output (success, stability) and
	// grasp parameters (wrist, joint).
	int numberOfGrasps;
	float buffer[1000];
	FILE * dataFP = fopen(planner->dataFile, "wb");

	// Write grasp data to a file
	char graspDataFile[1000];
	strcpy(graspDataFile, planner->baseFolder);
	strcat(graspDataFile, "/graspData.data");
	FILE * gdf = NULL;

	if (!boost::filesystem::exists(graspDataFile))
	{
		gdf = fopen(graspDataFile, "wb");
		fwrite(&numberOfGrasps, sizeof(int), 1, gdf);
	}

	// Write number of grasps
	int imgCounter = 0;

	for (int i = 0; i<planner->numberOfGrasps; i++)
	{

		// In the re-simulation environment, we do not write back those colliding with the table.
		if (!keepCollidingGrasps)
			if (!(planner->resultArr[i].counter))
				continue;

		// Get image from views and save into images folder
		if (!reSimulateFlag){
			std::string imDepth = std::string(viewFolder) + std::string(std::to_string(planner->resultArr[i].viewId)) + std::string(".png");
			std::string destImDepth = std::string(imageFolder) + std::string(std::to_string(imgCounter)) + std::string(".png");
			if (boost::filesystem::exists(imDepth)){
				boost::filesystem::copy_file(imDepth, destImDepth, boost::filesystem::copy_option::overwrite_if_exists);
			}
		}

		// Get object id, grasp params, output parameters etc print in a file.
		float success = 0;
		if (!(planner->resultArr[i].counter))
			success = -1;
		else success = planner->resultArr[i].successProbability/(double)planner->resultArr[i].counter;

		float stabilityArr[4];
		stabilityArr[0] = planner->resultArr[i].x1/(double)planner->resultArr[i].counter;
		stabilityArr[1] = planner->resultArr[i].r1/(double)planner->resultArr[i].counter;
		stabilityArr[2] = planner->resultArr[i].x2/(double)planner->resultArr[i].counter;
		stabilityArr[3] = planner->resultArr[i].r2/(double)planner->resultArr[i].counter;

		fwrite(&success, 4, 1, dataFP); // Writing success probability
		fwrite(stabilityArr, 4, 4, dataFP); // Writing stability values
		std::vector<float> tmpVec = planner->graspParams[i]; // TODO: graspParams is empty. We need to fill that in.
		float * tmpArr = new float[tmpVec.size()];
		for (int k = 0; k<tmpVec.size(); k++)
		{
			tmpArr[k] = planner->graspParams[i][k];
		}
		fwrite(tmpArr, 4, tmpVec.size(), dataFP); // Writing grasp parameters

		// Write grasp view and type data
		planner->resultArr[i].write(gdf);
		imgCounter++;
	}
	fclose(dataFP);
	if (gdf != NULL){
		fseek ( gdf , 0 , SEEK_SET );
		fwrite(&(planner->numberOfNoncollidingGrasps), sizeof(int), 1, gdf);
		fclose(gdf);
	}

	// If this is a re-simulation, then we do not need to upload any files.
	if (reSimulateFlag)
		return;

	// Test if we can run a command
	if (system(NULL)) puts ("Ok");
    	else exit (EXIT_FAILURE);

	// Remove views folder and unnecessary files
	if (boost::filesystem::exists(viewFolder))
		boost::filesystem::remove_all(viewFolder);
	if (boost::filesystem::exists(planner->debugLogFile))
		boost::filesystem::remove(planner->debugLogFile);
	if (boost::filesystem::exists(planner->depthFile))
		boost::filesystem::remove(planner->depthFile);

	// Write object id to a file
	char objectIdFile[1000];
	strcpy(objectIdFile, planner->baseFolder);
	strcat(objectIdFile, "/objectId.txt");
	FILE * f = fopen(objectIdFile, "w");
	fprintf(f, "%d\n", objectId);
	fclose(f);

	// Get model files and add them to the base folder
	char localModelFolder[1000], tmp[1000];
	strcpy(localModelFolder, planner->baseFolder);
	strcat(localModelFolder, "model/");
	boost::filesystem::create_directories(localModelFolder);

	// Copy relevant files
	boost::filesystem::path p(base);
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if ((i->path().string().find(planner->fileId) !=std::string::npos))
		{
			std::cout<<i->path().string()<<std::endl;
			try{
				strcpy(tmp, localModelFolder);
				strcat(tmp, i->path().filename().c_str());
				boost::filesystem::copy_file(i->path(), tmp, boost::filesystem::copy_option::overwrite_if_exists);
			}
			catch(const std::exception&)
			{}
		}
	}

	// Copy the object files
	char baseTmp[1000];
	strcpy(baseTmp, base);
	strcat(baseTmp, "/tmp");
	boost::filesystem::path p2(baseTmp);
	for (auto i = boost::filesystem::directory_iterator(p2); i != boost::filesystem::directory_iterator(); i++)
	{
		if ((i->path().string().find(planner->fileId) !=std::string::npos))
		{
			std::cout<<i->path().string()<<std::endl;
			try{
				strcpy(tmp, localModelFolder);
				strcat(tmp, i->path().filename().c_str());
				boost::filesystem::copy_file(i->path(), tmp, boost::filesystem::copy_option::overwrite_if_exists);
			}
			catch(const std::exception&)
			{}
		}
	}

    // Compress the folder
	char command[1000], zipFile[1000];
	strcpy(command, "cd ./tmp/data/");
	strcat(command, planner->fileId);
	strcat(command, " && zip -r ");
	strcat(command, planner->fileId);
	strcat(command, ".zip .");
	strcpy(zipFile, "./tmp/data/");
	strcat(zipFile, planner->fileId);
	strcat(zipFile, "/");
	strcat(zipFile, planner->fileId);
	strcat(zipFile, ".zip");

	// Call zip command
	system(command);

    // Create temp prefix
	char dropboxZipFile[1000];
    strcpy(dropboxZipFile, planner->dropboxFolder);
    strcat(dropboxZipFile, "/upload/");
    strcat(dropboxZipFile, planner->fileId);
    strcat(dropboxZipFile, ".zip");

    // Upload file
    bool uploadSuccess = Connector::UploadFile(zipFile);
    if (!uploadSuccess && boost::filesystem::exists(zipFile))
    {
        std::cout<<"Could not upload. Copying "<<zipFile<<" to "<<dropboxZipFile<<"."<<std::endl;
        boost::filesystem::copy_file(zipFile, dropboxZipFile, boost::filesystem::copy_option::overwrite_if_exists);
        boost::filesystem::remove(zipFile);
    }

}

void RemoveOldTmpFolders(const char * modelFolder, bool reSimulateFlag){

	// Remove all files from the model folder
	boost::filesystem::path p(modelFolder);
	time_t curTime = time(NULL);

	// Go over the files and delete them one by one.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 1800 &&
				((i->path().string().find("_include_light.xml") !=std::string::npos) ||
						(i->path().string().find("_include_table.xml") !=std::string::npos) ||
						(i->path().string().find("_Test.xml") !=std::string::npos) ||
						(i->path().string().find("_include_base_assets.xml") !=std::string::npos) ||
						(i->path().string().find("_include_base.xml") !=std::string::npos) ||
						(i->path().string().find("_include_object_assets.xml") !=std::string::npos) ||
						(i->path().string().find("_include_object.xml") !=std::string::npos))
		){
			try{
				boost::filesystem::remove_all(i->path());
			}
			catch(const std::exception&)
			{}
		}
	}

	char tmpModelFolder[1000];
	strcpy(tmpModelFolder, modelFolder);
	strcat(tmpModelFolder, "/tmp");

	// Remove all files from the temporary model folder
	if (boost::filesystem::exists(tmpModelFolder))
	{
		boost::filesystem::path p3(tmpModelFolder);

		// Go over the files and delete them one by one.
		for (auto i = boost::filesystem::directory_iterator(p3); i != boost::filesystem::directory_iterator(); i++)
		{
			time_t fileTime = boost::filesystem::last_write_time(i->path());
			if (curTime - fileTime > 1800 &&
					((i->path().string().find(".xml") !=std::string::npos)))
			{
				try{
					boost::filesystem::remove_all(i->path());
				}
				catch(const std::exception&)
				{}
			}
		}
	}

	if (reSimulateFlag)
		return;

	// Remove all files from the tmp folder
	boost::filesystem::path p2("./tmp/data");

	// Go over the files and delete them one by one.
	for (auto i = boost::filesystem::directory_iterator(p2); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 1800)
		{
			try{
				boost::filesystem::remove_all(i->path());
			}
			catch(const std::exception&)
			{}
		}
	}
}

}

Grasp::GraspResult * Grasp::readGraspData(const char * fileName){
	// Open file
	FILE * fp = fopen(fileName, "rb");

	// Error? Return
	if (fp == NULL)
		return NULL;

	// Allocate space
	int numberOfGrasps = 0;
	fread(&numberOfGrasps, sizeof(int), 1, fp);
	std::cout<<numberOfGrasps<<" grasp parameters are read!"<<std::endl;
	Grasp::GraspResult * newArr = new Grasp::GraspResult[numberOfGrasps];

	// Read each grasp
	for (int i = 0; i<numberOfGrasps; i++){
		newArr[i].read(fp);
	}

	// close fp
	fclose(fp);

	// return
	return newArr;
}

// File IO
void Grasp::GraspResult::write(FILE * &fp){
	// If no grasp data file, return.
	if (fp == NULL)
		return;

	fwrite(&viewId, sizeof(int), 1, fp);
	fwrite(&wpCount, sizeof(int), 1, fp);
	fwrite(&graspType, sizeof(int), 1, fp);
	float tmp[3];
	for (int i = 0; i<3; i++)
		tmp[i] = gazeDir[i];
	fwrite(tmp, sizeof(float), 3, fp);
	for (int i = 0; i<3; i++)
		tmp[i] = camPos[i];
	fwrite(tmp, sizeof(float), 3, fp);
}
void Grasp::GraspResult::read(FILE * &fp){

	if (fp == NULL){
		std::cerr<<"No grasp data file!"<<std::endl;
		return;
	}

	fread(&viewId, sizeof(int), 1, fp);
	fread(&wpCount, sizeof(int), 1, fp);
	fread(&graspType, sizeof(int), 1, fp);
	float tmp[3];
	fread(tmp, sizeof(float), 3, fp);
	for (int i = 0; i<3; i++)
		gazeDir[i] = tmp[i];
	fread(tmp, sizeof(float), 3, fp);
	for (int i = 0; i<3; i++)
		camPos[i] = tmp[i];
}

void Grasp::GraspResult::print(){
	std::cout<<"Grasp result"<<std::endl;
	std::cout<<"Success: "<<successProbability<<std::endl;
	std::cout<<"Stability: "<<x1<<" "<<r1<<" "<<x2<<" "<<r2<<" "<<std::endl;
	std::cout<<"Counters: "<<counter<<" "<<successCounter<<std::endl;
	std::cout<<"View id and grasp type: "<<viewId<<" "<<graspType<<std::endl;
	std::cout<<"Gaze dir and cam pos: "<<gazeDir[0]<<" "<<gazeDir[1]<<" "<<gazeDir[2]<<" "<<camPos[0]<<" "<<camPos[1]<<" "<<camPos[2]<<std::endl;
}

// PCATest.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"


const std::wstring horsePath = L"../../assets/horse-poses/horse-";

std::wstring getHorseFilePath(int poseNumber);
std::wstring getHorseEigenFilePath(int eigenVectorNumber);

int wmain(int argc, wchar_t *argv[])
{
	std::vector<double> coords;

	std::vector<std::string> connectivities;

	for (int i = 1; i <= 10; i++)
	{
		std::ifstream fileIn(::getHorseFilePath(i));

		std::string line;

		std::ostringstream connectivity;

		while (std::getline(fileIn, line))
		{
			if (line[0] == '#') //comment
				continue;

			std::string token;

			std::istringstream istr(line);
			istr >> token;

			if (token == "v")
			{
				double x, y, z;
				istr >> x >> y >> z;

				coords.push_back(x);
				coords.push_back(y);
				coords.push_back(z);
			}
			else if (token == "f")
			{
				connectivity << line << "\n";
			}
		}

		connectivities.push_back(connectivity.str());

	}

	Eigen::Map<Eigen::MatrixXd> horseMatrix{ coords.data(), (int)coords.size() / 10, 10 };

	Eigen::MatrixXd centered = horseMatrix.rowwise() - horseMatrix.colwise().mean();
	Eigen::MatrixXd cov = centered.adjoint() * centered;


	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);

	std::cout << eig.eigenvalues() << "\n\n";

	std::vector<double> transformedBasisArr;

	for (int i = 9; i >= 0; i--)
	{
		Eigen::VectorXd eigenvector = eig.eigenvectors().col(i);

		Eigen::VectorXd transformed = horseMatrix * eigenvector;

		std::ofstream fileOut(::getHorseEigenFilePath(10-i));

		fileOut << "# eigenvalue: " << eig.eigenvalues()(i) << "\n";

		for (int j = 0; j < transformed.size(); j += 3)
		{
			fileOut << "v " << transformed(j) << " " << transformed(j + 1) << " " << transformed(j + 2) << "\n";
		}

		fileOut << connectivities[0] << "\n";
		transformedBasisArr.insert(transformedBasisArr.end(), transformed.data(), transformed.data() + transformed.size());
	}

	Eigen::Map<Eigen::MatrixXd> transformedBasis{ transformedBasisArr.data(), (int)coords.size() / 10, 10 };

	Eigen::JacobiSVD<Eigen::MatrixXd> basisSVD{ transformedBasis, Eigen::ComputeFullU | Eigen::ComputeFullV };

	//Eigen::MatrixXd basisPinv = basisSVD.matrixV() * basisSVD.

	Eigen::MatrixXd horseHorseSpace = transformedBasis.inverse() * horseMatrix;

	std::cout << "Horses in horse space: \n\n " << horseHorseSpace.transpose() << "\n\n";

	Eigen::VectorXd customWeights{ 10 };
	customWeights <<0.40, 0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0;

	//customWeights << 32.0801, -2.05527, 0.826853, 56.8507, -33.7251, 84.1283, -92.3943, 60.4999, -523.026, 9487.12;

	const std::wstring customPath = horsePath + L"custom.obj";

	Eigen::VectorXd customMesh = Eigen::VectorXd::Zero(horseMatrix.innerSize());

	customMesh = transformedBasis * customWeights;

	{
		std::ofstream fileOut(customPath);

		//fileOut << "# weights: " << eig.eigenvalues()(i) << "\n";

		for (int j = 0; j < customMesh.size(); j += 3)
		{
			fileOut << "v " << customMesh(j) << " " << customMesh(j + 1) << " " << customMesh(j + 2) << "\n";
		}

		fileOut << connectivities[0] << "\n";
	}

    return 0;
}

std::wstring getHorseFilePath(int poseNumber)
{
	std::wostringstream path;
	path << horsePath << std::setw(2) << std::setfill(L'0') << poseNumber << L".obj";

	return path.str();
}

std::wstring getHorseEigenFilePath(int eigenVectorNumber)
{
	std::wostringstream path;
	path << horsePath << L"eigen-" << std::setw(2) << std::setfill(L'0') << eigenVectorNumber << L".obj";

	return path.str();
}
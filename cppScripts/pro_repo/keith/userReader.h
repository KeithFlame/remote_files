#pragma once
#ifndef USER_READER_H_
#define USER_READER_H_
#include "tinyxml.h"
#include <string>
#include <iostream>
#include "conventional_sign.h"


namespace user {

	struct UserReader
	{
		unsigned char serials;
		std::string name;
		std::string password;
		std::string password_tips;
		std::string password_protection;
		std::string password_protection_password;
	};
	
	class User {
	public:
		User();
		FileProcess addUserXml(UserReader user_reader, int& flag);
		FileProcess readUserXml();
		FileProcess checkUserXml(UserReader user_reader);
		FileProcess deleteUserXml(UserReader user_reader, int& flag);
		
	
		bool is_exist;					//�˻�����
		bool is_admin;					//����Ա�˻�
		bool is_authentication;			//ע���û�
		std::string password_tip;
	private:
		FileProcess fp;

	};


}

#endif // !USER_READER_H_

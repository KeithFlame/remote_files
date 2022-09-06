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
		
	
		bool is_exist;					//账户存在
		bool is_admin;					//管理员账户
		bool is_authentication;			//注册用户
		std::string password_tip;
	private:
		FileProcess fp;

	};


}

#endif // !USER_READER_H_

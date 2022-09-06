#include "userReader.h"
#pragma execution_character_set("utf-8")

using namespace user;

//初始化xml
user::User::User() :
    is_exist(false),
    is_admin(false),
    is_authentication(false),
    password_tip(""),
    fp(FileProcess::CAN_NOT_OPEN_FILE)
{

}

//验证程序是否可以运行，编写时为了编译通过，非测试用
FileProcess user::User::readUserXml()
{

    TiXmlDocument doc;//申明一个文档类型变量，用来存储读取的xml文档
    if (!doc.LoadFile("./conf/user_reader.xml"))   //检测xml文档是否存在
    {
        std::cerr << doc.ErrorDesc() << std::endl;
    }
    TiXmlElement* user_reader = doc.FirstChildElement();//指向xml文档的根元素


    if (user_reader == NULL)//检测根元素存在性     
    {
        std::cerr << "Failed to load file: No root element." << std::endl;
        doc.Clear();
    }
    else
    {
        TiXmlElement* mark_id = user_reader->FirstChildElement(); // xml的id 根节点下第一个子节点
        TiXmlElement* Chinese_name = mark_id->NextSiblingElement();        // mark_id 下一个子节点
        TiXmlElement* English_name = Chinese_name->NextSiblingElement();   // Chinese_name 下一个子节点
        TiXmlElement* Parent_ion = English_name->NextSiblingElement();   // English_name 下一个子节点
        TiXmlElement* DaughterNum = Parent_ion->NextSiblingElement();     // Parent_ion 下一个子节点
        TiXmlElement* Daughter_Ion = DaughterNum->NextSiblingElement();    // DaughterNum 下一个子节点
        TiXmlElement* Daughter = Daughter_Ion->FirstChildElement();//获得Daughter_ion的第一个子元素
        int temp = 0;

        while (mark_id != NULL)
        {
            temp++;
            std::cout << temp << std::endl;
            Chinese_name = mark_id->NextSiblingElement();        // mark_id 下一个子节点
            English_name = Chinese_name->NextSiblingElement();   // Chinese_name 下一个子节点
            Parent_ion = English_name->NextSiblingElement();   // English_name 下一个子节点
            DaughterNum = Parent_ion->NextSiblingElement();     // Parent_ion 下一个子节点
            Daughter_Ion = DaughterNum->NextSiblingElement();    // DaughterNum 下一个子节点
            Daughter = Daughter_Ion->FirstChildElement();    //获得Daughter_ion的第一个子元素
            std::string TypeMark_id = mark_id->Value();
            std::string Mark_idValue = mark_id->GetText();
            std::cout << TypeMark_id << " : " << Mark_idValue << std::endl;

            std::string TypeChinese_name = Chinese_name->Value();
            std::string Chinese_nameValue = Chinese_name->GetText();
            std::cout << TypeChinese_name << " : " << Chinese_nameValue << std::endl;

            std::string TypeEnglish_name = English_name->Value();
            std::string English_nameValue = English_name->GetText();
            std::cout << TypeEnglish_name << " : " << English_nameValue << std::endl;

            std::string TypeParent_ion = Parent_ion->Value();
            std::string Parent_ionValue = Parent_ion->GetText();
            std::cout << TypeParent_ion << " : " << Parent_ionValue << std::endl;

            std::string TypeDaughterNum = DaughterNum->Value();
            std::string DaughterNumValue = DaughterNum->GetText();
            std::cout << TypeDaughterNum << " : " << DaughterNumValue << std::endl;

            for (; Daughter != NULL; Daughter = Daughter->NextSiblingElement()) {
                std::string contactType = Daughter->Value();
                std::string contactValue = Daughter->GetText();
                std::cout << contactType << " : " << contactValue << std::endl;
            }
            mark_id = Daughter_Ion->NextSiblingElement();
        }
        std::cout << temp << std::endl;
    }
    fp = FileProcess::FILE_PROCESS_SUCCESS;
    return fp;
}

//添加用户
FileProcess user::User::addUserXml(UserReader user_reader, int& flag)
{
    TiXmlDocument doc;

    //检查文件是否打开
    if (!doc.LoadFile("./conf/users.xml"))
    {
        std::cerr <<"addUser: " << doc.ErrorDesc() << std::endl;
        fp = FileProcess::CAN_NOT_OPEN_FILE;
        return fp;
    }

    //获取根节点
    TiXmlElement* users = doc.RootElement();
    int account = 0;
    if (users == NULL)
    {
        std::cerr << "Failed to load file: No root element" << std::endl;
        doc.Clear();
        fp = FileProcess::FILE_IS_EMPTY;
        return fp;
    }
    else 
    {
        TiXmlElement* admin = users->FirstChildElement();
        TiXmlElement* serial_id = admin->FirstChildElement();
        TiXmlElement* name_id = serial_id->NextSiblingElement();
        
        while (admin != NULL)
        {
            account++;
            std::string name = name_id->GetText();
            if(name == user_reader.name)
            {
                flag = -1;
                fp = FileProcess::FILE_OPEN_UNNECCESARY;
                return fp;
            }
            admin = admin->NextSiblingElement();
            if (admin == NULL)
            {
                break;
            }
            serial_id = admin->FirstChildElement();
            name_id = serial_id->NextSiblingElement();
        }
    }

    if (account > 999)
    {
        flag = -2;
        fp = FileProcess::FILE_IS_EMPTY;
        return fp;
    }
        

    //添加新元素
    char buf[200];
    if(account<10)
        std::sprintf(buf, "user_00%d", account);
    else if(account < 100)
        std::sprintf(buf, "user_0%d", account);
    else
        std::sprintf(buf, "user_%d", account);
    //std::cout << buf << std::endl;
    TiXmlElement* new_user = new TiXmlElement(buf);

    TiXmlElement* serial_id = new TiXmlElement("serial_id");
    TiXmlText* serial_text = new TiXmlText(std::to_string(account).c_str());
    serial_id->LinkEndChild(serial_text);
    new_user->LinkEndChild(serial_id);

    TiXmlElement* name_id = new TiXmlElement("name_id");
    TiXmlText* name_text = new TiXmlText(user_reader.name.c_str());
    name_id->LinkEndChild(name_text);
    new_user->LinkEndChild(name_id);

    TiXmlElement* password_id = new TiXmlElement("password_id");
    TiXmlText* password_text = new TiXmlText(user_reader.password.c_str());
    password_id->LinkEndChild(password_text);
    new_user->LinkEndChild(password_id);

    TiXmlElement* password_tips_id = new TiXmlElement("password_tips_id");
    TiXmlText* password_tips_text = new TiXmlText(user_reader.password_tips.c_str());
    password_tips_id->LinkEndChild(password_tips_text);
    new_user->LinkEndChild(password_tips_id);

    TiXmlElement* password_protection_id = new TiXmlElement("password_protection_id");
    TiXmlText* password_protection_text = new TiXmlText(user_reader.password_protection.c_str());
    password_protection_id->LinkEndChild(password_protection_text);
    new_user->LinkEndChild(password_protection_id);

    TiXmlElement* password_protection_password_id = new TiXmlElement("password_protection_password_id");
    TiXmlText* password_protection_password_text = new TiXmlText(user_reader.password_protection_password.c_str());
    password_protection_password_id->LinkEndChild(password_protection_password_text);
    new_user->LinkEndChild(password_protection_password_id);

    users->LinkEndChild(new_user);

    bool result = doc.SaveFile("./conf/users.xml");
    
    if (result)
        flag = 0;
    else
        flag = 1;
    fp = FileProcess::FILE_PROCESS_SUCCESS;
    return fp;
    
}

//删除用户
FileProcess user::User::deleteUserXml(UserReader user_reader, int& flag)
{
    TiXmlDocument doc;
    //检查文件是否打开
    if (!doc.LoadFile("./conf/users.xml"))
    {
        std::cerr << doc.ErrorDesc() << std::endl;
        fp = FileProcess::CAN_NOT_OPEN_FILE;
        flag = -1;
        return fp;
    }

    TiXmlElement* users = doc.FirstChildElement();

    //检测文件中是否有内容
    if (users == NULL)
    {
        std::cerr << "Failed to load file: No root element" << std::endl;
        doc.Clear();
        fp = FileProcess::FILE_IS_EMPTY;
        flag = -1;
        return fp;
    }
    else
    {

        TiXmlElement* user_id = users->FirstChildElement();
        TiXmlElement* serial_id = user_id->FirstChildElement();
        TiXmlElement* name_id = serial_id->NextSiblingElement();
        TiXmlElement* password_id = name_id->NextSiblingElement();

        //遍历中
        while (user_id != NULL)
        {            
            serial_id = user_id->FirstChildElement();
            name_id = serial_id->NextSiblingElement();
            password_id = name_id->NextSiblingElement();

            std::string name = name_id->Value();
            std::string password = password_id->Value();
            auto is_true = [](std::string a, std::string b)->bool
            {
                if (a == b)
                    return 1;
                else
                    return 0;
            };
            if (user_reader.name == name)
            {
                auto rc = users->RemoveChild(user_id->ToElement());
                if (rc)
                    flag = 1;
                else
                    flag = 0;
                break;
            }
            flag = -1;

        }

        fp = FileProcess::FILE_PROCESS_SUCCESS;
        return fp;
    }
}

//检查账户信息是否正确
FileProcess user::User::checkUserXml(UserReader user_reader)
{
    TiXmlDocument doc;

    //检查文件是否打开
    if (!doc.LoadFile("./conf/users.xml"))
    {
        std::cerr << doc.ErrorDesc() << std::endl;
        fp = FileProcess::CAN_NOT_OPEN_FILE;
        return fp;
    }

    TiXmlElement* users = doc.FirstChildElement();

    //检测文件中是否有内容
    if (users == NULL)
    {
        std::cerr << "Failed to load file: No root element" << std::endl;
        doc.Clear();
        fp = FileProcess::FILE_IS_EMPTY;
        return fp;
    }
    else
    {

        TiXmlElement* admin = users->FirstChildElement();
        TiXmlElement* serial_id = admin->FirstChildElement();
        TiXmlElement* name_id = serial_id->NextSiblingElement();
        TiXmlElement* password_id = name_id->NextSiblingElement();
        TiXmlElement* password_tips_id = password_id->NextSiblingElement();

        while (admin != NULL)
        {
            std::string name = name_id->GetText();
            std::string password = password_id->GetText();
            auto is_true = [](std::string a, std::string b)->bool
            {
                if (a == b)
                    return 1;
                else
                    return 0;
            };
            int res1 = is_true(user_reader.name, name);
            int res2 = res1 * is_true(user_reader.password, password);
            int res3 = res2 * is_true(user_reader.name, "admin");

            //判断是否是注册用户以及是否为管理员
            switch (res1 + res2 + res3)
            {
            case 0:
                {
                    is_exist = false;
                    is_authentication = false;
                    is_admin = false;
                    break;
                }
            case 1:
                {
                    is_exist = true;
                    is_authentication = false;
                    is_admin = false;
                    break;
                }
            case 2:
                {
                    is_exist = true;
                    is_authentication = true;
                    is_admin = false;
                    break;
                }
            case 3:
                {
                    is_exist = true;
                    is_authentication = true;
                    is_admin = true;
                    break;
                }
            default:
                {

                }
            }


            //检测到就跳出循环
            if (is_exist)
            {
                password_tip = password_tips_id->GetText();
                break;
            }
                

            admin = admin->NextSiblingElement();
            if (admin != NULL)
            {
                serial_id = admin->FirstChildElement();
                name_id = serial_id->NextSiblingElement();
                password_id = name_id->NextSiblingElement();
                password_tips_id = password_id->NextSiblingElement();
            }
        }
        fp = FileProcess::FILE_PROCESS_SUCCESS;
        return fp;
    }
}

//修改密码








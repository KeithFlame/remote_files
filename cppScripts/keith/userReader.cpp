#include "userReader.h"
#pragma execution_character_set("utf-8")

using namespace user;

//��ʼ��xml
user::User::User() :
    is_exist(false),
    is_admin(false),
    is_authentication(false),
    password_tip(""),
    fp(FileProcess::CAN_NOT_OPEN_FILE)
{

}

//��֤�����Ƿ�������У���дʱΪ�˱���ͨ�����ǲ�����
FileProcess user::User::readUserXml()
{

    TiXmlDocument doc;//����һ���ĵ����ͱ����������洢��ȡ��xml�ĵ�
    if (!doc.LoadFile("./conf/user_reader.xml"))   //���xml�ĵ��Ƿ����
    {
        std::cerr << doc.ErrorDesc() << std::endl;
    }
    TiXmlElement* user_reader = doc.FirstChildElement();//ָ��xml�ĵ��ĸ�Ԫ��


    if (user_reader == NULL)//����Ԫ�ش�����     
    {
        std::cerr << "Failed to load file: No root element." << std::endl;
        doc.Clear();
    }
    else
    {
        TiXmlElement* mark_id = user_reader->FirstChildElement(); // xml��id ���ڵ��µ�һ���ӽڵ�
        TiXmlElement* Chinese_name = mark_id->NextSiblingElement();        // mark_id ��һ���ӽڵ�
        TiXmlElement* English_name = Chinese_name->NextSiblingElement();   // Chinese_name ��һ���ӽڵ�
        TiXmlElement* Parent_ion = English_name->NextSiblingElement();   // English_name ��һ���ӽڵ�
        TiXmlElement* DaughterNum = Parent_ion->NextSiblingElement();     // Parent_ion ��һ���ӽڵ�
        TiXmlElement* Daughter_Ion = DaughterNum->NextSiblingElement();    // DaughterNum ��һ���ӽڵ�
        TiXmlElement* Daughter = Daughter_Ion->FirstChildElement();//���Daughter_ion�ĵ�һ����Ԫ��
        int temp = 0;

        while (mark_id != NULL)
        {
            temp++;
            std::cout << temp << std::endl;
            Chinese_name = mark_id->NextSiblingElement();        // mark_id ��һ���ӽڵ�
            English_name = Chinese_name->NextSiblingElement();   // Chinese_name ��һ���ӽڵ�
            Parent_ion = English_name->NextSiblingElement();   // English_name ��һ���ӽڵ�
            DaughterNum = Parent_ion->NextSiblingElement();     // Parent_ion ��һ���ӽڵ�
            Daughter_Ion = DaughterNum->NextSiblingElement();    // DaughterNum ��һ���ӽڵ�
            Daughter = Daughter_Ion->FirstChildElement();    //���Daughter_ion�ĵ�һ����Ԫ��
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

//����û�
FileProcess user::User::addUserXml(UserReader user_reader, int& flag)
{
    TiXmlDocument doc;

    //����ļ��Ƿ��
    if (!doc.LoadFile("./conf/users.xml"))
    {
        std::cerr <<"addUser: " << doc.ErrorDesc() << std::endl;
        fp = FileProcess::CAN_NOT_OPEN_FILE;
        return fp;
    }

    //��ȡ���ڵ�
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
        

    //�����Ԫ��
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

//ɾ���û�
FileProcess user::User::deleteUserXml(UserReader user_reader, int& flag)
{
    TiXmlDocument doc;
    //����ļ��Ƿ��
    if (!doc.LoadFile("./conf/users.xml"))
    {
        std::cerr << doc.ErrorDesc() << std::endl;
        fp = FileProcess::CAN_NOT_OPEN_FILE;
        flag = -1;
        return fp;
    }

    TiXmlElement* users = doc.FirstChildElement();

    //����ļ����Ƿ�������
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

        //������
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

//����˻���Ϣ�Ƿ���ȷ
FileProcess user::User::checkUserXml(UserReader user_reader)
{
    TiXmlDocument doc;

    //����ļ��Ƿ��
    if (!doc.LoadFile("./conf/users.xml"))
    {
        std::cerr << doc.ErrorDesc() << std::endl;
        fp = FileProcess::CAN_NOT_OPEN_FILE;
        return fp;
    }

    TiXmlElement* users = doc.FirstChildElement();

    //����ļ����Ƿ�������
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

            //�ж��Ƿ���ע���û��Լ��Ƿ�Ϊ����Ա
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


            //��⵽������ѭ��
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

//�޸�����








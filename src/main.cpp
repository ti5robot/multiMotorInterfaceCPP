/*多电机，位置接口，状态接口，暂停接口，恢复接口 C++版*/
#include "Ti5BASIC.h"
using namespace std;
int main()
{
    allocate_variable(6);

    login();
    cout << "登录成功" << endl;
    get_elc_info(6);
    int q = get_elektrische_Maschinen_status(6);
    clear_elc_error(6);
    set_elc_info(ampere, 0, MotorTypeHelper::REG_TARGET_CURRENT, 0);
    TH.j[0]=1.2;
    move_to_joint();
    deallocate_variable();
    logout();

    return 0;
}

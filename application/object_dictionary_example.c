#include <stdio.h>
#include "object_dictionary.h"

int example_object_dictionary() {
   // 创建对象字典
   ObjectDictionary* od = OD_Create();
   if (od == NULL) {
    //    printf("创建对象字典失败\n");
       return 1;
   }

   // 创建一些示例条目
   OD_Entry* entry1 = (OD_Entry*)malloc(sizeof(OD_Entry));
   entry1->index = 0x1000;
   entry1->subindex = 0;
   entry1->dataType = OD_TYPE_INT32;
   entry1->accessRead = true;
   entry1->accessWrite = true;
   entry1->next = NULL;

   OD_Entry* entry2 = (OD_Entry*)malloc(sizeof(OD_Entry));
   entry2->index = 0x1001;
   entry2->subindex = 0;
   entry2->dataType = OD_TYPE_STRING;
   entry2->accessRead = true;
   entry2->accessWrite = true;
   entry2->next = NULL;

   // 添加条目
   if (!OD_AddEntry(od, entry1)) {
    //    printf("添加条目1失败\n");
       free(entry1);
       OD_Destroy(od);
       return 1;
   }

   if (!OD_AddEntry(od, entry2)) {
    //    printf("添加条目2失败\n");
       free(entry2);
       OD_Destroy(od);
       return 1;
   }

   // 设置值
   int32_t value1 = 12345;
   if (!OD_SetValue(od, 0x1000, 0, &value1)) {
    //    printf("设置值1失败\n");
       OD_Destroy(od);
       return 1;
   }

   const char* value2 = "Hello, Object Dictionary!";
   if (!OD_SetValue(od, 0x1001, 0, (void*)value2)) {
    //    printf("设置值2失败\n");
       OD_Destroy(od);
       return 1;
   }

   // 打印所有条目
   OD_PrintAll(od);

   // 读取值
   int32_t readValue1;
   if (OD_GetValue(od, 0x1000, 0, &readValue1)) {
    //    printf("读取值1: %d\n", readValue1);
   }

   char readValue2[256];
   if (OD_GetValue(od, 0x1001, 0, readValue2)) {
    //    printf("读取值2: %s\n", readValue2);
   }

   // 删除一个条目
   if (OD_RemoveEntry(od, 0x1000, 0)) {
    //    printf("成功删除条目1\n");
   }

   // 再次打印所有条目
   OD_PrintAll(od);

   // 清理
   OD_Destroy(od);
   return 0;
}
#ifndef   _ZIGBEE_WRT_PROTOCOL_H_
#define   _ZIGBEE_WRT_PROTOCOL_H_
/*
	20150216  BCG
*/


typedef enum
{
	NORMAL_STATUS = 0,
	UN_NORMAL_STATUS = 1,

}STATUS_;

typedef enum
{
	OK = 0,
	ERR = 1,
}xxx;

typedef enum
{
	ZGB_WRT_ACK = 0x0001,
	WRT_ZGB_ACK = 0x8001,
	
	WRT_ZGB_DATA_CMD = 0x8002,//下行透传
	ZGB_WRT_DATA_CMD = 0x0002,//上行透传
	
	
	WRT_ZGB_SET_INFO_CMD = 0x8003,//
	WRT_ZGB_GET_INFO_CMD = 0x8004,//无消息体
	ZGB_WRT_ACK_INFO_CMD = 0x0003,//应答信息
		
}ccccc;
typedef struct
{
	uint16   head;//0x7e7e
	uint8    msg_serial_num;//流水号
	uint16   msg_cmd;//msg_id
	uint16   msg_len;//消息体长度
	uint8    crc8;//校验
}Zg_Header_Struct;

typedef struct
{
	uint8   src_type;//来源类型
//	uint16  end_dev_addr;// zigbee short address
	afAddrType_t	end_dev_addr;// zigbee  address
	uint16  len;//透传数据
	//uint8   *pdata;
}Msg_Zg_WRT_transfer;//上行下行都用此数据结构 数据紧接在此结构体之后

typedef struct
{
	uint16  panid;// zigbee panid
	uint16  profileid;
	uint8   endpoint;
	uint8   Reserved[6];
}Msg_WRT_Zg_Set_Get_info;//上行下行都用此数据结构 


//Report_Status  rpst;
//rpst.end_dev_nums = 0;
//char  buf[128];
//int   buflen;
//buflen = pack_msg(0x01, &rpst, sizeof(rpst), buf);
//send(buf, buflen);


int pack_msg(uint16 cmdid, char *pdata, int len, char *pout);

int unpack_msg(char *pdata, int len);


int  pack_msg_transfer(Msg_Zg_WRT_transfer *mzwt,uint8 *pdata,uint16 dataLen, char *pout);
void deal_data(char *pData, uint16 dataLen);

#endif
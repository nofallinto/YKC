模块(含.c .h文件)描述
main,BoardSupport	: 硬件相关文件，rtos初始化. 和TI不同的是main包含了ST自动生成的硬件初始化代码
DrvST 				: 把ST的硬件驱动转换成类TI的模式

特别注意（置顶）！！！特别注意（置顶）！！！特别注意（置顶）！！！：
每次代码生成后需要手动修改：每次代码生成后需要手动修改：每次代码生成后需要手动修改：
1，ld链接文件需要替换回生成前的备份
2，删除LinkResources
3，为了让LWIP能使用自己修改版DHCP代码，需要做如下修改：
	\Middlewares\Third_Party\LwIP\src\core\ipv4\ip4.c

		A，	将struct netif * ip4_route(const ip4_addr_t *dest)
			/* 主要为DHCP做此修改，否则当自身没设置IP时，会被判定无法路由 */
			if ((netif_default == NULL) || !netif_is_up(netif_default) || !netif_is_link_up(netif_default) ||
			     ip4_addr_isany_val(*netif_ip4_addr(netif_default)) || ip4_addr_isloopback(dest)) {
			
			中的if所在行改成：
	  			if ((netif_default == NULL) || !netif_is_up(netif_default) || !netif_is_link_up(netif_default) || ip4_addr_isloopback(dest)) {
	
		B，	将static int ip4_input_accept(struct netif *netif)
			  /* 主要为DHCP做此修改，否则当自身没设置IP时，此包会被忽略 */
			  if ((netif_is_up(netif)) && (!ip4_addr_isany_val(*netif_ip4_addr(netif)))) {
			
			中的if所在行换成：
		  		if (netif_is_up(netif)) {
		  		
		  		
		  	
2024.1.13
1，串口收发全改用DMA（崔烨鸿）
2，改进了GPRS通信速率（崔烨鸿）
3，增加了一种GPRS错误类型（崔烨鸿）
4，将MqttPubReq的定义改到了更合理的DrvST里（崔烨鸿）
5，增加MQTT测试代码（崔烨鸿）	  	
		  		
2024.1.8
1、添加gprs,水泵，红外等测试代码（舒行雷）
2、添加另外一边的红外驱动（舒行雷）
		  		
2024.12.15
1，独立了正式版测试板板子逻辑文件：MdlYKC、MdlTest（崔烨鸿）

2024.12.01 
1，编译器查找路径增加yLib，避免引用时写路径（崔烨鸿）
2，应用测试宏作为硬件测试的宏（崔烨鸿）

2024.11.27
1，完善硬件测试程序、（舒行雷）

2024.10.1 
1，增加软件I2C（刘永康）



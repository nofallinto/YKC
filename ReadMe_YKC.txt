ģ��(��.c .h�ļ�)����
main,BoardSupport	: Ӳ������ļ���rtos��ʼ��. ��TI��ͬ����main������ST�Զ����ɵ�Ӳ����ʼ������
DrvST 				: ��ST��Ӳ������ת������TI��ģʽ

�ر�ע�⣨�ö����������ر�ע�⣨�ö����������ر�ע�⣨�ö�����������
ÿ�δ������ɺ���Ҫ�ֶ��޸ģ�ÿ�δ������ɺ���Ҫ�ֶ��޸ģ�ÿ�δ������ɺ���Ҫ�ֶ��޸ģ�
1��ld�����ļ���Ҫ�滻������ǰ�ı���
2��ɾ��LinkResources
3��Ϊ����LWIP��ʹ���Լ��޸İ�DHCP���룬��Ҫ�������޸ģ�
	\Middlewares\Third_Party\LwIP\src\core\ipv4\ip4.c

		A��	��struct netif * ip4_route(const ip4_addr_t *dest)
			/* ��ҪΪDHCP�����޸ģ���������û����IPʱ���ᱻ�ж��޷�·�� */
			if ((netif_default == NULL) || !netif_is_up(netif_default) || !netif_is_link_up(netif_default) ||
			     ip4_addr_isany_val(*netif_ip4_addr(netif_default)) || ip4_addr_isloopback(dest)) {
			
			�е�if�����иĳɣ�
	  			if ((netif_default == NULL) || !netif_is_up(netif_default) || !netif_is_link_up(netif_default) || ip4_addr_isloopback(dest)) {
	
		B��	��static int ip4_input_accept(struct netif *netif)
			  /* ��ҪΪDHCP�����޸ģ���������û����IPʱ���˰��ᱻ���� */
			  if ((netif_is_up(netif)) && (!ip4_addr_isany_val(*netif_ip4_addr(netif)))) {
			
			�е�if�����л��ɣ�
		  		if (netif_is_up(netif)) {
		  		
		  		
		  	
2024.1.13
1�������շ�ȫ����DMA�����Ǻ裩
2���Ľ���GPRSͨ�����ʣ����Ǻ裩
3��������һ��GPRS�������ͣ����Ǻ裩
4����MqttPubReq�Ķ���ĵ��˸������DrvST����Ǻ裩
5������MQTT���Դ��루���Ǻ裩	  	
		  		
2024.1.8
1�����gprs,ˮ�ã�����Ȳ��Դ��루�����ף�
2���������һ�ߵĺ��������������ף�
		  		
2024.12.15
1����������ʽ����԰�����߼��ļ���MdlYKC��MdlTest�����Ǻ裩

2024.12.01 
1������������·������yLib����������ʱд·�������Ǻ裩
2��Ӧ�ò��Ժ���ΪӲ�����Եĺ꣨���Ǻ裩

2024.11.27
1������Ӳ�����Գ��򡢣������ף�

2024.10.1 
1���������I2C����������



import requests
# by pip install beautifulsoup4
from bs4 import BeautifulSoup
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
import time
my_uri = 'https://www.hermes.com/tw/zh/'


###### request ######
# headers = {
#     "accept": "text/html,application/xhtml+xml,application/xml;q=0.9,image/avif,image/webp,image/apng,*/*;q=0.8,application/signed-exchange;v=b3;q=0.9",
#     "accept-encoding": "gzip, deflate, br",
#     "accept-language": "zh-TW,zh;q=0.9,en-US;q=0.8,en;q=0.7",
#     "cache-control": "max-age=0",
#     "cookie": "has_js=1; x-xsrf-token=f8d9cbc4-3635-4fc4-8372-7544e3f14176; _gcl_au=1.1.584630519.1688978905; _gid=GA1.2.1674371932.1688978905; correlation_id=4z23efxq3w7vtp1fv3t31p3rgofbfhu28ra5fgh43zbxmyomig1a07jun1jl9fy8; _cs_mk=0.2572086147003414_1688981747949; ECOM_SESS=bzqzhoenql0n30oilyp75be35g; _gcl_aw=GCL.1688981809.CjwKCAjw2K6lBhBXEiwA5RjtCRw2XFQcr3a54h3QRhWbF4vgT5k2UhsD3NvIqMSdQrwLuDiVUMOzKBoCm9kQAvD_BwE; _gac_UA-64545050-1=1.1688981814.CjwKCAjw2K6lBhBXEiwA5RjtCRw2XFQcr3a54h3QRhWbF4vgT5k2UhsD3NvIqMSdQrwLuDiVUMOzKBoCm9kQAvD_BwE; _ga=GA1.2.463121754.1688978905; _ga_Y862HCHCQ7=GS1.1.1688981741.2.1.1688982906.0.0.0; datadome=7Xk67BE69goDTn55cOeBlkUlBSAJFB-PXmDfPBTbgbrBqUNnnIrp0-TiZ7_iBObE8EiG2d0TPYdWEtKpyLGaawsu47uSQhu4da~pTyLp5zEGDec5XGPGV2tbZ26uxxVj",
#     "if-none-match": "W/\"1688981432-0-gzip\"",
#     "referer": "https://www.hermes.com/tw/zh/",
#     "sec-ch-device-memory": "8",
#     "sec-ch-ua": "\" Not A;Brand\";v=\"99\", \"Chromium\";v=\"101\", \"Google Chrome\";v=\"101\"",
#     "sec-ch-ua-arch": "\"x86\"",
#     "sec-ch-ua-full-version-list": "\" Not A;Brand\";v=\"99.0.0.0\", \"Chromium\";v=\"101.0.4951.64\", \"Google Chrome\";v=\"101.0.4951.64\"",
#     "sec-ch-ua-mobile": "?0",
#     "sec-ch-ua-model": "\"\"",
#     "sec-ch-ua-platform": "\"Linux\"",
#     "sec-fetch-dest": "document",
#     "sec-fetch-mode": "navigate",
#     "sec-fetch-site": "same-origin",
#     "sec-fetch-user": "?1",    
#     "upgrade-insecure-requests": "1",
#     "user-agent": "Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/101.0.0.0 Safari/537.36"
# }
# response = requests.get(url=my_uri, headers=headers)
# soup = BeautifulSoup(response.text, "html.parser")

# print(soup.prettify()) 

# print(response)
# print(response.text)


###### selenium ######
proxy = '58.58.213.55:8888'
# proxy = '183.95.80.102:8080',
# proxy = '123.160.31.71:8080',
# proxy = '115.231.128.79:8080',
# proxy = '166.111.77.32:80',
# proxy = '43.240.138.31:8080',
# proxy = '218.201.98.196:3128'


options = Options()
user_agent = 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/101.0.0.0 Safari/537.36'

# user_agent = "Mozilla/5.0 (Windows NT 6.3; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36"
# user_agent = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_9_2) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/35.0.1916.153 Safari/537.36"
# user_agent = "Mozilla/5.0 (Windows NT 6.1; WOW64; rv:30.0) Gecko/20100101 Firefox/30.0"
# user_agent = "Mozilla/5.0 (Macintosh; Intel Mac OS X 10_9_2) AppleWebKit/537.75.14 (KHTML, like Gecko) Version/7.0.3 Safari/537.75.14"
# user_agent = "Mozilla/5.0 (compatible; MSIE 10.0; Windows NT 6.2; Win64; x64; Trident/6.0)"
# user_agent = "Mozilla/5.0 (Windows; U; Windows NT 5.1; it; rv:1.8.1.11) Gecko/20071127 Firefox/2.0.0.11"
# user_agent = "Opera/9.25 (Windows NT 5.1; U; en)"
# user_agent = "Mozilla/4.0 (compatible; MSIE 6.0; Windows NT 5.1; SV1; .NET CLR 1.1.4322; .NET CLR 2.0.50727)"
# user_agent = "Mozilla/5.0 (compatible; Konqueror/3.5; Linux) KHTML/3.5.5 (like Gecko) (Kubuntu)"
# user_agent = "Mozilla/5.0 (X11; U; Linux i686; en-US; rv:1.8.0.12) Gecko/20070731 Ubuntu/dapper-security Firefox/1.5.0.12"
# user_agent = "Lynx/2.8.5rel.1 libwww-FM/2.14 SSL-MM/1.4.1 GNUTLS/1.2.9"
# user_agent = "Mozilla/5.0 (X11; Linux i686) AppleWebKit/535.7 (KHTML, like Gecko) Ubuntu/11.04 Chromium/16.0.912.77 Chrome/16.0.912.77 Safari/535.7"
# user_agent = "Mozilla/5.0 (X11; Ubuntu; Linux i686; rv:10.0) Gecko/20100101 Firefox/10.0"

driver_path = '/home/ron/fusion_ws/src/all_process/script/chromedriver'

arg = ("--user-agent={}, --disable-notifications, --proxy-server={}, --disable-gpu, --hide-scrollbars, --disable-javascript".format(user_agent, proxy))

# options.add_argument('--disable-notifications')
# options.add_argument('--user-agent=%s' % user_agent)
# options.add_argument('--proxy-server=' + proxy)
options.add_argument(arg)



#docker
# chrome = webdriver.Chrome('/home/ron/work/src/all_process/script/chromedriver', chrome_options=options)
#real

# print( options)
# for i in range(0,5):
#     print(i)
chrome = webdriver.Chrome(driver_path, chrome_options=options)

chrome.get(my_uri)

print(chrome.page_source)





